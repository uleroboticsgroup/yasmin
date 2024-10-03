// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <algorithm>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/yasmin_logs.hpp"

using namespace yasmin;

StateMachine::StateMachine(std::vector<std::string> outcomes)
    : State(outcomes) {
  this->current_state_mutex = std::make_unique<std::mutex>();
}

void StateMachine::add_state(std::string name, std::shared_ptr<State> state,
                             std::map<std::string, std::string> transitions) {

  this->states.insert({name, state});
  this->transitions.insert({name, transitions});

  if (this->start_state.empty()) {
    this->start_state = name;
  }
}

void StateMachine::add_state(std::string name, std::shared_ptr<State> state) {
  this->add_state(name, state, {});
}

void StateMachine::set_start_state(std::string state_name) {
  this->start_state = state_name;
}

std::string StateMachine::get_start_state() { return this->start_state; }

void StateMachine::cancel_state() {
  State::cancel_state();

  const std::lock_guard<std::mutex> lock(*this->current_state_mutex.get());
  if (!this->current_state.empty()) {
    this->states.at(this->current_state)->cancel_state();
  }
}

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

std::string
StateMachine::execute(std::shared_ptr<blackboard::Blackboard> blackboard) {

  this->current_state_mutex->lock();
  this->current_state = this->start_state;
  this->current_state_mutex->unlock();

  std::map<std::string, std::string> transitions;
  std::string outcome;

  while (true) {

    this->current_state_mutex->lock();

    auto state = this->states.at(this->current_state);
    transitions = this->transitions.at(this->current_state);
    this->current_state_mutex->unlock();

    outcome = (*state.get())(blackboard);

    // check outcome belongs to state
    if (std::find(state->get_outcomes().begin(), state->get_outcomes().end(),
                  outcome) == this->outcomes.end()) {
      throw std::logic_error("Outcome (" + outcome +
                             ") is not register in state " +
                             this->current_state);
    }

    // translate outcome using transitions
    if (transitions.find(outcome) != transitions.end()) {

      YASMIN_LOG_INFO("%s: %s --> %s", this->current_state.c_str(),
                      outcome.c_str(), transitions.at(outcome).c_str());

      outcome = transitions.at(outcome);
    }

    // outcome is an outcome of the sm
    if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) !=
        this->outcomes.end()) {

      this->current_state_mutex->lock();
      this->current_state.clear();
      this->current_state_mutex->unlock();

      return outcome;

      // outcome is a state
    } else if (this->states.find(outcome) != this->states.end()) {

      this->current_state_mutex->lock();
      this->current_state = outcome;
      this->current_state_mutex->unlock();

      // outcome is not in the sm
    } else {
      throw std::logic_error("Outcome (" + outcome + ") without transition");
    }
  }

  return "";
}

std::string StateMachine::execute() {

  std::shared_ptr<blackboard::Blackboard> blackboard =
      std::make_shared<blackboard::Blackboard>();

  std::string outcome = this->operator()(blackboard);
  return outcome;
}

std::string StateMachine::operator()() {
  std::shared_ptr<blackboard::Blackboard> blackboard =
      std::make_shared<blackboard::Blackboard>();

  return this->operator()(blackboard);
}

std::string StateMachine::to_string() {

  std::string result = "State Machine\n";

  for (const auto &s : this->get_states()) {
    result += s.first + " (" + s.second->to_string() + ")\n";
    for (const auto &t : this->transitions.at(s.first)) {
      result += "\t" + t.first + " --> " + t.second + "\n";
    }
  }

  return result;
}