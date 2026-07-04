// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include "yasmin/state_machine.hpp"

#include <algorithm>
#include <csignal>
#include <exception>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "yasmin/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"
#include "yasmin/state_utils.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

namespace {
std::sig_atomic_t sigint_caught = 0;

extern "C" void sigint_handler(int) { sigint_caught = 1; }

bool was_sigint_caught() {
  auto r = sigint_caught;
  sigint_caught = 0;
  return r;
}
} // namespace

StateMachine::StateMachine(const Outcomes &outcomes, bool handle_sigint)
    : StateMachine("", outcomes, handle_sigint) {}

StateMachine::StateMachine(const std::string &name, const Outcomes &outcomes,
                           bool handle_sigint)
    : State(outcomes), current_state_mutex(std::make_unique<std::mutex>()),
      name(name) {
  this->set_sigint_handler(handle_sigint);
}

StateMachine::~StateMachine() {
  this->states.clear();
  this->transitions.clear();
  this->remappings.clear();
  this->parameter_mappings.clear();
}

void StateMachine::add_state(const std::string &name, State::SharedPtr state,
                             const Transitions &transitions,
                             const Remappings &remappings,
                             const ParameterMappings &parameter_mappings) {

  if (this->states.find(name) != this->states.end()) {
    throw std::logic_error("State '" + name +
                           "' already registered in the state machine");
  }

  if (this->outcomes.find(name) != this->outcomes.end()) {
    throw std::logic_error("State name '" + name +
                           "' is already registered as an outcome");
  }

  for (auto it = transitions.begin(); it != transitions.end(); ++it) {
    const std::string &key = it->first;
    const std::string &value = it->second;

    if (key.empty()) {
      throw std::invalid_argument("Transitions with empty source in state '" +
                                  name + "'");
    }

    if (value.empty()) {
      throw std::invalid_argument("Transitions with empty target in state '" +
                                  name + "'");
    }

    const auto &state_outcomes = state->get_outcomes();
    if (state_outcomes.find(key) == state_outcomes.end()) {
      std::ostringstream oss;
      oss << "State '" << name << "' references unregistered outcomes '" << key
          << "', available outcomes are [";
      for (const auto &outcome : state_outcomes) {
        oss << "'" << outcome << "'";
        if (outcome != *state_outcomes.rbegin()) {
          oss << ", ";
        }
      }
      oss << "]";
      throw std::invalid_argument(oss.str());
    }
  }

  std::ostringstream transitions_oss;

  for (auto const &t : transitions) {
    transitions_oss << "\n\t" << t.first << " --> " << t.second;
  }

  YASMIN_LOG_DEBUG("Adding state '%s' of type '%s' with transitions: %s",
                   name.c_str(), state->to_string().c_str(),
                   transitions_oss.str().c_str());

  this->states.insert({name, std::move(state)});
  this->transitions.insert({name, transitions});
  this->remappings.insert({name, remappings});
  this->parameter_mappings.insert({name, parameter_mappings});

  if (this->start_state.empty()) {
    this->set_start_state(name);
  }

  // Mark state machine as no validated
  this->validated.store(false);
  this->configured.store(false);
}

void StateMachine::set_start_state(const std::string &state_name) {

  if (state_name.empty()) {
    throw std::invalid_argument("Initial state cannot be empty");

  } else if (this->states.find(state_name) == this->states.end()) {
    throw std::invalid_argument("Initial state '" + state_name +
                                "' is not in the state machine");
  }

  YASMIN_LOG_DEBUG("Setting start state to '%s'", state_name.c_str());

  this->start_state = state_name;

  // Mark state machine as no validated
  this->validated.store(false);
  this->configured.store(false);
}

void StateMachine::set_parameter_mappings(
    const std::string &state_name,
    const ParameterMappings &parameter_mappings) {

  if (this->states.find(state_name) == this->states.end()) {
    throw std::invalid_argument("State '" + state_name +
                                "' is not in the state machine");
  }

  this->parameter_mappings[state_name] = parameter_mappings;
  this->configured.store(false);
}

const ParameterMappingsMap &
StateMachine::get_parameter_mappings() const noexcept {
  return this->parameter_mappings;
}

void StateMachine::apply_parameter_mappings(const std::string &state_name,
                                            const State::SharedPtr &state) {
  yasmin::apply_parameter_mappings("State machine", this->parameter_mappings,
                                   state_name, *this, state);
}

void StateMachine::configure() {
  if (yasmin::check_already_configured(this->configured, "State machine",
                                       this->to_string().c_str()))
    return;

  for (const auto &[state_name, state] : this->states) {
    this->apply_parameter_mappings(state_name, state);
    state->configure();
  }

  this->configured.store(true);
}

std::string StateMachine::wait_for_current_state() {
  std::unique_lock<std::mutex> lock(*this->current_state_mutex.get());

  if (!this->current_state_cond.wait_for(
          lock, std::chrono::seconds(5), [this]() {
            return !this->current_state.empty() ||
                   !this->execution_active.load() ||
                   this->cancel_state_machine_requested.load();
          })) {
    YASMIN_LOG_WARN("Timeout waiting for current state in '%s'",
                    this->to_string().c_str());
  }

  return this->current_state;
}

void StateMachine::throw_if_cancel_state_machine_requested() {
  if (!this->cancel_state_machine_requested.load()) {
    return;
  }

  this->execution_active.store(false);
  this->set_current_state("");
  this->current_state_cond.notify_all();
  throw StateMachineCancelException(this->to_string());
}

std::string const &StateMachine::get_start_state() const noexcept {
  return this->start_state;
}

StateMap const &StateMachine::get_states() const noexcept {
  return this->states;
}

TransitionsMap const &StateMachine::get_transitions() const noexcept {
  return this->transitions;
}

std::string StateMachine::get_current_state() const {
  const std::lock_guard<std::mutex> lock(*this->current_state_mutex.get());
  return this->current_state;
}

void StateMachine::set_current_state(const std::string &state_name) {
  const std::lock_guard<std::mutex> lock(*this->current_state_mutex.get());
  this->current_state = state_name;
  this->current_state_cond.notify_all();
}

void StateMachine::add_start_cb(StartCallbackType cb) {
  this->start_cbs.emplace_back(cb);
}

void StateMachine::add_transition_cb(TransitionCallbackType cb) {
  this->transition_cbs.emplace_back(cb);
}

void StateMachine::add_end_cb(EndCallbackType cb) {
  this->end_cbs.emplace_back(cb);
}

void StateMachine::call_start_cbs(Blackboard::SharedPtr blackboard,
                                  const std::string &start_state) {

  try {
    for (const auto &callback : this->start_cbs) {
      callback(blackboard, start_state);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute start callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_transition_cbs(Blackboard::SharedPtr blackboard,
                                       const std::string &from_state,
                                       const std::string &to_state,
                                       const std::string &outcome) {

  try {
    for (const auto &callback : this->transition_cbs) {
      callback(blackboard, from_state, to_state, outcome);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute transition callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_end_cbs(Blackboard::SharedPtr blackboard,
                                const std::string &outcome) {

  try {
    for (const auto &callback : this->end_cbs) {
      callback(blackboard, outcome);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute end callback: %s",
                     std::string(e.what()).c_str());
  }
}

Remappings
StateMachine::compose_remappings(const Remappings &parent_remappings,
                                 const Remappings &state_remappings) {
  Remappings composed_remappings = parent_remappings;

  for (const auto &[state_key, state_target] : state_remappings) {
    auto parent_it = parent_remappings.find(state_target);
    if (parent_it != parent_remappings.end()) {
      composed_remappings[state_key] = parent_it->second;
    } else {
      composed_remappings[state_key] = state_target;
    }
  }

  return composed_remappings;
}

void StateMachine::validate(bool strict_mode) {

  YASMIN_LOG_DEBUG("Validating state machine '%s'", this->to_string().c_str());

  if (this->validated.load() && !strict_mode) {
    YASMIN_LOG_DEBUG("State machine '%s' has already been validated",
                     this->to_string().c_str());
    return;
  }

  // Check initial state
  if (this->start_state.empty()) {
    throw std::runtime_error("No initial state set");
  }

  // Terminal outcomes from all transitions
  std::set<std::string> terminal_outcomes;

  // Check all states
  for (auto it = this->states.begin(); it != this->states.end(); ++it) {

    const std::string &state_name = it->first;
    const State::SharedPtr &state = it->second;
    Transitions transitions = this->transitions.at(state_name);

    Outcomes outcomes = state->get_outcomes();

    if (strict_mode) {
      // Check if all outcomes of the state are in transitions
      const auto &sm_outcomes = this->get_outcomes();
      for (const std::string &o : outcomes) {

        if (transitions.find(o) == transitions.end() &&
            sm_outcomes.find(o) == sm_outcomes.end()) {

          throw std::runtime_error("State '" + state_name + "' outcome '" + o +
                                   "' not registered in transitions");

          // Outcomes of the state that are in outcomes of the state machine
          // do not need transitions
        } else if (sm_outcomes.find(o) != sm_outcomes.end()) {
          terminal_outcomes.insert(o);
        }
      }
    }

    // If state is a state machine, concurrence, or orthogonal state, validate
    // it. Use get_inner_state() to unwrap proxy wrappers (e.g.
    // PythonStateHolder) so that Python-loaded states are correctly identified.
    auto *inner = state->get_inner_state();
    if (auto *sm = dynamic_cast<StateMachine *>(inner)) {
      sm->validate(strict_mode);
    } else if (auto *c = dynamic_cast<Concurrence *>(inner)) {
      c->validate(strict_mode);
    } else if (auto *o = dynamic_cast<OrthogonalState *>(inner)) {
      o->validate(strict_mode);
    }

    // Add terminal outcomes
    for (auto it = transitions.begin(); it != transitions.end(); ++it) {
      const std::string &value = it->second;
      terminal_outcomes.insert(value);
    }
  }

  // Check terminal outcomes for the state machine
  const auto &sm_outcomes = this->get_outcomes();

  if (strict_mode) {
    // Check if all outcomes from the state machine are in the terminal outcomes
    for (const std::string &o : sm_outcomes) {
      if (terminal_outcomes.find(o) == terminal_outcomes.end()) {
        throw std::runtime_error("Target outcome '" + o +
                                 "' not registered in transitions");
      }
    }
  }

  // Check if all terminal outcomes are states or outcomes of the state machine
  for (const std::string &o : terminal_outcomes) {
    if (this->states.find(o) == this->states.end() &&
        sm_outcomes.find(o) == sm_outcomes.end()) {
      throw std::runtime_error("State machine outcome '" + o +
                               "' not registered as outcome neither state");
    }
  }

  // State machine has been validated
  this->validated.store(true);
}

std::string StateMachine::execute(Blackboard::SharedPtr blackboard) {

  this->validate();
  this->configure();

  YASMIN_LOG_INFO("Executing state machine with initial state '%s'",
                  this->start_state.c_str());
  this->call_start_cbs(blackboard, this->start_state);

  this->cancel_state_machine_requested.store(false);
  this->execution_active.store(true);
  this->set_current_state(this->start_state);

  bool state_machine_ends = false;
  std::string outcome;

  try {
    while (!state_machine_ends) {
      if (was_sigint_caught()) {
        this->cancel_state_machine();
      }
      this->throw_if_cancel_state_machine_requested();

      std::string current_state = this->get_current_state();
      outcome =
          this->execute_step(blackboard, current_state, state_machine_ends);
    }
  } catch (...) {
    this->execution_active.store(false);
    this->set_current_state("");
    this->current_state_cond.notify_all();
    throw;
  }

  this->execution_active.store(false);
  this->current_state_cond.notify_all();
  return outcome;
}

std::string StateMachine::execute() {
  Blackboard::SharedPtr blackboard = yasmin::Blackboard::make_shared();

  return this->execute(blackboard);
}

std::string StateMachine::operator()() {
  Blackboard::SharedPtr blackboard = yasmin::Blackboard::make_shared();

  return this->operator()(blackboard);
}

void StateMachine::cancel_state() {

  if (this->is_running()) {
    const auto current_state = this->wait_for_current_state();

    if (!current_state.empty()) {
      this->states.at(current_state)->cancel_state();
    }
  }
}

void StateMachine::cancel_state_machine() {

  if (this->is_running()) {
    YASMIN_LOG_INFO("Canceling state machine '%s'", this->to_string().c_str());

    this->cancel_state_machine_requested.store(true);
    this->current_state_cond.notify_all();

    const auto current_state = this->wait_for_current_state();

    if (!current_state.empty()) {
      const auto &state = this->states.at(current_state);
      if (auto child_state_machine =
              std::dynamic_pointer_cast<StateMachine>(state)) {
        child_state_machine->cancel_state_machine();
      } else {
        state->cancel_state();
      }
    }

    State::cancel_state();
  }
}

void StateMachine::set_sigint_handler(bool handle) {
  struct sigaction sigint_action;
  sigint_action.sa_handler = handle ? sigint_handler : SIG_DFL;
  sigemptyset(&sigint_action.sa_mask);
  sigint_action.sa_flags = 0;
  sigaction(SIGINT, &sigint_action, nullptr);
}

std::string StateMachine::execute_step(Blackboard::SharedPtr blackboard,
                                       const std::string &current_state,
                                       bool &state_machine_ends) {

  const auto &states = this->states;
  const auto &local_outcomes = this->outcomes;

  auto state_it = states.find(current_state);
  if (state_it == states.end()) {
    throw std::logic_error("Active state '" + current_state +
                           "' not found in state machine");
  }
  const auto &state = state_it->second;
  const auto &local_transitions = this->transitions.at(current_state);
  const auto &local_remappings = this->remappings.at(current_state);

  auto parent_remappings = blackboard->get_remappings();
  auto composed_remappings =
      StateMachine::compose_remappings(parent_remappings, local_remappings);
  blackboard->set_remappings(composed_remappings);

  std::string outcome;
  std::string old_outcome;

  try {
    outcome = (*state.get())(blackboard);
  } catch (...) {
    blackboard->set_remappings(parent_remappings);
    throw;
  }
  old_outcome = outcome;

  blackboard->set_remappings(parent_remappings);

  this->throw_if_cancel_state_machine_requested();

  const auto &state_outcomes = state->get_outcomes();
  if (state_outcomes.find(outcome) == state_outcomes.end()) {
    throw std::logic_error("Outcome '" + outcome +
                           "' is not registered in state " + current_state);
  }

  if (local_transitions.find(outcome) != local_transitions.end()) {
    outcome = local_transitions.at(outcome);
  }

  YASMIN_LOG_INFO("State machine transitioning '%s' : '%s' --> '%s'",
                  current_state.c_str(), old_outcome.c_str(), outcome.c_str());

  if (local_outcomes.find(outcome) != local_outcomes.end()) {
    this->set_current_state("");
    YASMIN_LOG_INFO("State machine ends with outcome '%s'", outcome.c_str());
    this->call_end_cbs(blackboard, outcome);
    state_machine_ends = true;
  } else if (states.find(outcome) != states.end()) {
    this->call_transition_cbs(blackboard, current_state, outcome, old_outcome);
    this->set_current_state(outcome);
  } else {
    throw std::logic_error("Outcome '" + outcome +
                           "' is not a state nor a state machine outcome");
  }

  return outcome;
}

std::string StateMachine::to_string() const {
  return "State Machine [" +
         yasmin::join(this->get_states(), ", ",
                      [](const auto &p) {
                        return p.first + " (" + p.second->to_string() + ")";
                      }) +
         "]";
}
