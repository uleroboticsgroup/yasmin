
#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

#include "yasmin/state_machine.hpp"

using namespace yasmin;

StateMachine::StateMachine(std::vector<std::string> outcomes)
    : State(outcomes) {}

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

std::string StateMachine::get_current_state() { return this->current_state; }

std::string
StateMachine::execute(std::shared_ptr<blackboard::Blackboard> blackboard) {

  this->current_state = this->start_state;
  std::map<std::string, std::string> transitions =
      this->transitions.at(this->start_state);
  std::string outcome;

  while (true) {
    outcome = (*this->states.at(this->current_state))(blackboard);

    // tranlate outcome using transitions
    if (transitions.find(outcome) != transitions.end()) {
      outcome = transitions.at(outcome);
    }

    // outcome is an outcome of the sm
    if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) !=
        this->outcomes.end()) {

      this->current_state.clear();
      return outcome;

      // outcome is a state
    } else if (this->states.find(outcome) != this->states.end()) {

      this->current_state = outcome;
      transitions = this->transitions.at(this->current_state);

      // outcome is not in the sm
    } else {
      throw "Outcome (" + outcome + ") without transition";
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