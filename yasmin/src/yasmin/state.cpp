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
#include <string>
#include <vector>
#include <exception>

#include "yasmin/state.hpp"

using namespace yasmin;

State::State(std::vector<std::string> outcomes) { this->outcomes = outcomes; }

std::string
State::operator()(std::shared_ptr<blackboard::Blackboard> blackboard) {
  this->canceled = false;
  std::string outcome = this->execute(blackboard);

  if (std::find(this->outcomes.begin(), this->outcomes.end(), outcome) ==
      this->outcomes.end()) {
    throw std::logic_error("Outcome (" + outcome + ") does not exist");
  }

  return outcome;
}

void State::cancel_state() { this->canceled = true; }

bool State::is_canceled() { return this->canceled; }

std::vector<std::string> const &State::get_outcomes() { return this->outcomes; }
