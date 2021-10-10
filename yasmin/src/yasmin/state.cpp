
#include <algorithm>
#include <string>
#include <vector>

#include "yasmin/state.hpp"

using namespace yasmin;

State::State(std::vector<std::string> outcomes) { this->outcomes = outcomes; }

std::string
State::operator()(std::shared_ptr<blackboard::Blackboard> blackboard) {
  this->canceled = false;
  std::string outcome = this->execute(blackboard);

  if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) ==
      this->outcomes.end()) {
    throw "Outcome (" + outcome + ") does not exist";
  }

  return outcome;
}

void State::cancel_state() { this->canceled = true; }

bool State::is_canceled() { return this->canceled; }

std::vector<std::string> const &State::get_outcomes() { return this->outcomes; }
