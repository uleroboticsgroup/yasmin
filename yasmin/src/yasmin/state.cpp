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
#include <stdexcept>
#include <string>
#include <vector>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"

using namespace yasmin;

State::State(std::set<std::string> outcomes) : outcomes(outcomes) {}

std::string
State::operator()(std::shared_ptr<blackboard::Blackboard> blackboard) {

  YASMIN_LOG_DEBUG("Executing state '%s'", this->to_string().c_str());

  this->canceled.store(false);
  this->running.store(true);
  std::string outcome = this->execute(blackboard);

  if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) ==
      this->outcomes.end()) {

    std::string outcomes_string = "[";
    auto outcomes = this->get_outcomes();

    for (auto it = outcomes.begin(); it != outcomes.end(); ++it) {
      const auto &s = *it;
      outcomes_string += s;

      if (std::next(it) != outcomes.end()) {
        outcomes_string += ", ";
      }
    }

    outcomes_string += "]";

    throw std::logic_error(
        "Outcome '" + outcome +
        "' does not belong to the outcomes of "
        "the state '" +
        this->to_string() +
        "'. The possible outcomes are: " + outcomes_string.c_str());
  }

  this->running.store(false);
  return outcome;
}

bool State::is_canceled() const { return this->canceled.load(); }

bool State::is_running() const { return this->running.load(); }

std::set<std::string> const &State::get_outcomes() { return this->outcomes; }
