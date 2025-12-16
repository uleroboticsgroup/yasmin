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
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __GNUG__     // If using GCC/G++
#include <cxxabi.h> // For abi::__cxa_demangle
#endif

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

State::State(const Outcomes &outcomes) : outcomes(outcomes) {
  if (outcomes.empty()) {
    throw std::logic_error("A state must have at least one possible outcome.");
  }
}

void State::set_status(StateStatus new_status) {
  this->status.store(new_status);
}

StateStatus State::get_status() const { return this->status.load(); }

bool State::is_idle() const noexcept {
  return this->get_status() == StateStatus::IDLE;
}

bool State::is_running() const noexcept {
  return this->get_status() == StateStatus::RUNNING;
}

bool State::is_canceled() const noexcept {
  return this->get_status() == StateStatus::CANCELED;
}

bool State::is_completed() const noexcept {
  return this->get_status() == StateStatus::COMPLETED;
}

std::string State::operator()(Blackboard::SharedPtr blackboard) {

  YASMIN_LOG_DEBUG("Executing state '%s'", this->to_string().c_str());
  this->set_status(StateStatus::RUNNING);

  // Execute the specific logic of the state
  std::string outcome = this->execute(blackboard);

  // Check if the outcome is valid
  if (this->outcomes.find(outcome) == this->outcomes.end()) {

    // Construct a string representation of the possible outcomes
    std::string outcomes_string = "[";
    const auto &outcomes = this->get_outcomes();

    for (auto it = outcomes.begin(); it != outcomes.end(); ++it) {
      const auto &s = *it;
      outcomes_string += s;

      // Add a comma if this is not the last element
      if (std::next(it) != outcomes.end()) {
        outcomes_string += ", ";
      }
    }

    outcomes_string += "]";

    // Mark as idle before throwing exception
    this->set_status(StateStatus::IDLE);

    // Throw an exception if the outcome is not valid
    throw std::logic_error("Outcome '" + outcome +
                           "' does not belong to the outcomes of "
                           "the state '" +
                           this->to_string() +
                           "'. The possible outcomes are: " + outcomes_string);
  }

  // Mark as completed if not canceled
  if (!this->is_canceled()) {
    this->set_status(StateStatus::COMPLETED);
  }

  return outcome; // Return the valid outcome
}

Outcomes const &State::get_outcomes() const noexcept { return this->outcomes; }

std::string State::to_string() const {
  std::string name = typeid(*this).name();

#ifdef __GNUG__ // If using GCC/G++
  int status;
  // Demangle the name using GCC's demangling function
  char *demangled =
      abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
  if (status == 0) {
    name = demangled;
  }
  free(demangled);
#endif

  return name; // Return the demangled class name
}