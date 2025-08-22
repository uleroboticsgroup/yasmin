// Copyright (C) 2023 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <algorithm>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

using namespace yasmin;

StateMachine::StateMachine(std::set<std::string> outcomes)
    : State(outcomes), current_state_mutex(std::make_unique<std::mutex>()) {}

void StateMachine::add_state(std::string name, std::shared_ptr<State> state,
                             std::map<std::string, std::string> transitions,
                             std::map<std::string, std::string> remapping) {

  if (this->states.find(name) != this->states.end()) {
    throw std::logic_error("State '" + name +
                           "' already registered in the state machine");
  }

  if (std::find(this->outcomes.begin(), this->outcomes.end(), name) !=
      this->outcomes.end()) {
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

    if (std::find(state->get_outcomes().begin(), state->get_outcomes().end(),
                  key) == state->get_outcomes().end()) {
      std::ostringstream oss;
      oss << "State '" << name << "' references unregistered outcomes '" << key
          << "', available outcomes are ";
      for (const auto &outcome : state->get_outcomes()) {
        oss << outcome << " ";
      }
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

  this->states.insert({name, state});
  this->transitions.insert({name, transitions});
  this->remappings.insert({name, remapping});

  if (this->start_state.empty()) {
    this->set_start_state(name);
  }

  // Mark state machine as no validated
  this->validated.store(false);
}

void StateMachine::set_start_state(std::string state_name) {

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
}

std::string StateMachine::get_start_state() { return this->start_state; }

std::map<std::string, std::shared_ptr<State>> const &
StateMachine::get_states() {
  return this->states;
}

std::map<std::string, std::map<std::string, std::string>> const &
StateMachine::get_transitions() {
  return this->transitions;
}

std::string StateMachine::get_current_state() {
  const std::lock_guard<std::mutex> lock(*this->current_state_mutex.get());
  return this->current_state;
}

void StateMachine::set_current_state(std::string state_name) {
  const std::lock_guard<std::mutex> lock(*this->current_state_mutex.get());
  this->current_state = state_name;
  this->current_state_cond.notify_all();
}

void StateMachine::add_start_cb(StartCallbackType cb,
                                std::vector<std::string> args) {
  this->start_cbs.emplace_back(cb, args);
}

void StateMachine::add_transition_cb(TransitionCallbackType cb,
                                     std::vector<std::string> args) {
  this->transition_cbs.emplace_back(cb, args);
}

void StateMachine::add_end_cb(EndCallbackType cb,
                              std::vector<std::string> args) {
  this->end_cbs.emplace_back(cb, args);
}

void StateMachine::call_start_cbs(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    const std::string &start_state) {

  try {
    for (const auto &callback_pair : this->start_cbs) {
      const auto &cb = callback_pair.first;
      const auto &args = callback_pair.second;
      cb(blackboard, start_state, args);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute start callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_transition_cbs(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    const std::string &from_state, const std::string &to_state,
    const std::string &outcome) {

  try {
    for (const auto &callback_pair : this->transition_cbs) {
      const auto &cb = callback_pair.first;
      const auto &args = callback_pair.second;
      cb(blackboard, from_state, to_state, outcome, args);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute transition callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_end_cbs(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    const std::string &outcome) {

  try {
    for (const auto &callback_pair : this->end_cbs) {
      const auto &cb = callback_pair.first;
      const auto &args = callback_pair.second;
      cb(blackboard, outcome, args);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute end callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::validate(bool strict_mode) {

  YASMIN_LOG_DEBUG("Validating state machine '%s'", this->to_string().c_str());

  if (this->validated.load() && !strict_mode) {
    YASMIN_LOG_DEBUG("State machine '%s' has already been validated",
                     this->to_string().c_str());
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
    const std::shared_ptr<State> &state = it->second;
    std::map<std::string, std::string> transitions =
        this->transitions.at(state_name);

    std::set<std::string> outcomes = state->get_outcomes();

    if (strict_mode) {
      // Check if all outcomes of the state are in transitions
      for (const std::string &o : outcomes) {

        if (transitions.find(o) == transitions.end() &&
            std::find(this->get_outcomes().begin(), this->get_outcomes().end(),
                      o) == this->get_outcomes().end()) {

          throw std::runtime_error("State '" + state_name + "' outcome '" + o +
                                   "' not registered in transitions");

          // Outcomes of the state that are in outcomes of the state machine
          // do not need transitions
        } else if (std::find(this->get_outcomes().begin(),
                             this->get_outcomes().end(),
                             o) != this->get_outcomes().end()) {
          terminal_outcomes.insert(o);
        }
      }
    }

    // If state is a state machine, validate it
    if (std::dynamic_pointer_cast<StateMachine>(state)) {
      std::dynamic_pointer_cast<StateMachine>(state)->validate(strict_mode);
    }

    // Add terminal outcomes
    for (auto it = transitions.begin(); it != transitions.end(); ++it) {
      const std::string &value = it->second;
      terminal_outcomes.insert(value);
    }
  }

  // Check terminal outcomes for the state machine
  std::set<std::string> sm_outcomes(this->get_outcomes().begin(),
                                    this->get_outcomes().end());

  if (strict_mode) {
    // Check if all outcomes from the state machine are in the terminal outcomes
    for (const std::string &o : this->get_outcomes()) {
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
                               "' not registered as outcome or state");
    }
  }

  // State machine has been validated
  this->validated.store(true);
}

std::string
StateMachine::execute(std::shared_ptr<blackboard::Blackboard> blackboard) {

  this->validate();

  YASMIN_LOG_INFO("Executing state machine with initial state '%s'",
                  this->start_state.c_str());
  this->call_start_cbs(blackboard, this->start_state);

  this->set_current_state(this->start_state);

  std::map<std::string, std::string> transitions;
  std::map<std::string, std::string> remapping;
  std::string outcome;
  std::string old_outcome;

  while (!this->is_canceled()) {

    std::string current_state = this->get_current_state();
    auto state = this->states.at(current_state);
    transitions = this->transitions.at(current_state);
    remapping = this->remappings.at(current_state);
    blackboard->set_remapping(remapping);

    outcome = (*state.get())(blackboard);
    old_outcome = std::string(outcome);

    // Check outcome belongs to state
    if (std::find(state->get_outcomes().begin(), state->get_outcomes().end(),
                  outcome) == state->get_outcomes().end()) {
      throw std::logic_error("Outcome '" + outcome +
                             "' is not registered in state " +
                             this->current_state);
    }

    // Translate outcome using transitions
    if (transitions.find(outcome) != transitions.end()) {
      outcome = transitions.at(outcome);
    }

    // Outcome is an outcome of the sm
    if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) !=
        this->outcomes.end()) {

      this->set_current_state("");
      YASMIN_LOG_INFO("State machine ends with outcome '%s'", outcome.c_str());
      this->call_end_cbs(blackboard, outcome);

      return outcome;

      // Outcome is a state
    } else if (this->states.find(outcome) != this->states.end()) {

      YASMIN_LOG_INFO("State machine transitioning '%s' : '%s' --> '%s'",
                      this->current_state.c_str(), old_outcome.c_str(),
                      outcome.c_str());
      this->call_transition_cbs(blackboard, this->current_state, outcome,
                                old_outcome);

      this->set_current_state(outcome);

      // Outcome is not in the sm
    } else {
      throw std::logic_error("Outcome '" + outcome +
                             "' is not a state nor a state machine outcome");
    }
  }

  throw std::runtime_error("Ending canceled state machine '" +
                           this->to_string() + "' with bad transition");
}

std::string StateMachine::execute() {

  std::shared_ptr<blackboard::Blackboard> blackboard =
      std::make_shared<blackboard::Blackboard>();

  return this->operator()(blackboard);
}

std::string StateMachine::operator()() {
  std::shared_ptr<blackboard::Blackboard> blackboard =
      std::make_shared<blackboard::Blackboard>();

  return this->operator()(blackboard);
}

void StateMachine::cancel_state() {
  State::cancel_state();

  if (this->is_running()) {

    auto current_state = this->get_current_state();

    while (current_state.empty()) {
      std::unique_lock<std::mutex> lock(*this->current_state_mutex.get());
      this->current_state_cond.wait(lock);
      current_state = this->get_current_state();
    }

    if (!current_state.empty()) {
      this->states.at(current_state)->cancel_state();
    }
  }
}

std::string StateMachine::to_string() {

  std::ostringstream oss;
  oss << "StateMachine [";

  const auto &states = this->get_states();

  for (auto it = states.begin(); it != states.end(); ++it) {
    const auto &s = *it;
    oss << s.first << " (" << s.second->to_string() << ")";

    if (std::next(it) != states.end()) {
      oss << ", ";
    }
  }

  oss << "]";

  return oss.str();
}
