// Copyright (C) 2025 Georgia Tech Research Institute
// Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "yasmin/concurrence.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "yasmin/logs.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/state_utils.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

Concurrence::Concurrence(const StateMap &states,
                         const std::string &default_outcome,
                         const OutcomeMap &outcome_map,
                         const ParameterMappingsMap &parameter_mappings)
    : State(yasmin::generate_possible_outcomes(outcome_map, default_outcome)),
      states(states), default_outcome(default_outcome),
      outcome_map(outcome_map), parameter_mappings(parameter_mappings) {

  // Require at least one state
  if (states.empty()) {
    throw std::invalid_argument("Map of concurrent states cannot be empty");
  }

  // Check for duplicate state instances
  std::set<std::shared_ptr<State>> unique_instances;
  for (const auto &[state_name, state] : states) {
    if (unique_instances.find(state) != unique_instances.end()) {
      throw std::invalid_argument(
          "There are duplicate state instances in the states");
    }
    unique_instances.insert(state);
  }

  // Validate outcome map and prepare intermediate outcomes map
  for (const auto &[outcome, requirements] : outcome_map) {
    if (requirements.empty()) {
      throw std::invalid_argument(
          "Outcome '" + outcome +
          "' in the outcome map must have at least one requirement");
    }
    for (const auto &[state_name, intermediate_outcome] : requirements) {
      // Check if state name exists in the states map
      auto state_it = states.find(state_name);
      if (state_it == states.end()) {
        throw std::invalid_argument(
            "State name '" + state_name +
            "' in the outcome map under outcome '" + outcome +
            "' is not provided in the concurrent set of states to run");
      }

      // Check if intermediate outcome is valid for the state
      State::SharedPtr state = state_it->second;
      if (state->get_outcomes().find(intermediate_outcome) ==
          state->get_outcomes().end()) {
        throw std::invalid_argument(
            "Intermediate outcome '" + intermediate_outcome +
            "' under outcome '" + outcome +
            "' of the outcome map is not a valid outcome of state '" +
            state->to_string() + "'");
      }

      this->intermediate_outcome_map.insert({state_name, ""});
    }
  }
}

void Concurrence::set_parameter_mappings(
    const std::string &state_name,
    const ParameterMappings &parameter_mappings) {
  if (this->states.find(state_name) == this->states.end()) {
    throw std::invalid_argument(
        "State name '" + state_name +
        "' is not provided in the concurrent set of states to run");
  }

  this->parameter_mappings[state_name] = parameter_mappings;
  this->configured.store(false);
}

const ParameterMappingsMap &
Concurrence::get_parameter_mappings() const noexcept {
  return this->parameter_mappings;
}

void Concurrence::apply_parameter_mappings(const std::string &state_name,
                                           const State::SharedPtr &state) {
  yasmin::apply_parameter_mappings("Concurrence", this->parameter_mappings,
                                   state_name, *this, state);
}

void Concurrence::configure() {
  if (yasmin::check_already_configured(this->configured, "Concurrence",
                                       this->to_string().c_str()))
    return;

  for (const auto &[state_name, state] : this->states) {
    this->apply_parameter_mappings(state_name, state);
    state->configure();
  }

  this->configured.store(true);
}

std::string Concurrence::execute(Blackboard::SharedPtr blackboard) {
  this->configure();
  std::vector<std::thread> state_threads;

  // Reset stored intermediate outcomes before starting a new execution.
  for (auto &[state_name, intermediate_outcome] :
       this->intermediate_outcome_map) {
    (void)state_name;
    intermediate_outcome.clear();
  }

  // Initialize the parallel execution of all the states.
  // Each branch receives a blackboard copy that shares the underlying storage
  // but keeps an isolated remapping scope.
  for (const auto &[state_name, state] : this->states) {
    Blackboard::SharedPtr thread_blackboard =
        std::make_shared<Blackboard>(*blackboard);
    state_threads.push_back(std::thread([this, state_name, state,
                                         thread_blackboard]() {
      std::string outcome = (*state.get())(thread_blackboard);
      const std::lock_guard<std::mutex> lock(this->intermediate_outcome_mutex);
      this->intermediate_outcome_map[state_name] = outcome;
    }));
  }

  // Wait for states to finish
  for (std::thread &state_thread : state_threads) {
    if (state_thread.joinable()) {
      state_thread.join();
    }
  }

  // Handle a cancel
  if (this->is_canceled()) {
    return this->default_outcome;
  }

  // Build final outcome
  Outcomes satisfied_outcomes = yasmin::evaluate_satisfied_outcomes(
      this->outcome_map,
      [this](const std::string &state_name) -> const std::string & {
        auto it = this->intermediate_outcome_map.find(state_name);
        if (it == this->intermediate_outcome_map.end() || it->second.empty()) {
          throw std::runtime_error("An intermediate outcome for state '" +
                                   state_name + "' was not received.");
        }
        return it->second;
      });

  return yasmin::resolve_outcome(satisfied_outcomes, this->default_outcome,
                                 "concurrent state execution (" +
                                     this->to_string() + ")");
}

void Concurrence::cancel_state() {
  for (const auto &[state_name, state] : this->states) {
    state->cancel_state();
  }
  yasmin::State::cancel_state();
}

const StateMap &Concurrence::get_states() const noexcept {
  return this->states;
}

const OutcomeMap &Concurrence::get_outcome_map() const noexcept {
  return this->outcome_map;
}

const std::string &Concurrence::get_default_outcome() const noexcept {
  return this->default_outcome;
}

void Concurrence::validate(bool strict_mode) {
  for (const auto &[state_name, state] : this->states) {
    (void)state_name;
    // Use get_inner_state() to unwrap proxy wrappers (e.g. PythonStateHolder)
    // so that Python-loaded states are correctly identified.
    auto *inner = state->get_inner_state();
    if (auto *sm = dynamic_cast<StateMachine *>(inner)) {
      sm->validate(strict_mode);
    } else if (auto *c = dynamic_cast<Concurrence *>(inner)) {
      c->validate(strict_mode);
    } else if (auto *o = dynamic_cast<OrthogonalState *>(inner)) {
      o->validate(strict_mode);
    }
  }
}

std::string Concurrence::to_string() const {
  return "Concurrence [" +
         yasmin::join(this->states, ", ",
                      [](const auto &p) {
                        return p.first + " (" + p.second->to_string() + ")";
                      }) +
         "]";
}
