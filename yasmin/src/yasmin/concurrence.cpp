// Copyright (C) 2025 Georgia Tech Research Institute
// Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
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

#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"

using namespace yasmin;

Concurrence::Concurrence(std::set<std::shared_ptr<State>> states,
                         std::string default_outcome, OutcomeMap outcome_map)
    : State(generate_possible_outcomes(outcome_map, default_outcome)),
      states(states), default_outcome(default_outcome),
      outcome_map(outcome_map) {

  // Require at least one state
  if (states.empty()) {
    throw std::invalid_argument("Set of concurrent states cannot be empty");
  }

  // Validate outcome map and prepare intemedaite outcomes map
  for (const auto &[outcome, requirements] : outcome_map) {
    if (requirements.empty()) {
      throw std::invalid_argument(
          "Outcome '" + outcome +
          "' in the outcome map must have at least one requirement");
    }
    for (const auto &[state, intermediate_outcome] : requirements) {
      if (state->get_outcomes().find(intermediate_outcome) ==
          state->get_outcomes().end()) {
        throw std::invalid_argument(
            "Intemedaite outcome '" + intermediate_outcome +
            "' under outcome '" + outcome +
            "' of the outcome map is not a valid outcome of state '" +
            state->to_string() + "'");
      }
      if (states.find(state) == states.end()) {
        throw std::invalid_argument(
            "State '" + state->to_string() +
            "' in the outcome map under outcome '" + outcome +
            "' is not provided in the concurrent set of states to run");
      }
      intermediate_outcome_map.insert({state, nullptr});
    }
  }
}

std::string
Concurrence::execute(std::shared_ptr<blackboard::Blackboard> blackboard) {
  std::vector<std::thread> state_threads;

  // Initialize the parallel execution of all the states
  for (std::shared_ptr<State> state : states) {
    state_threads.push_back(std::thread([this, state, blackboard]() {
      std::string outcome = (*state.get())(blackboard);
      const std::lock_guard<std::mutex> lock(this->intermediate_outcome_mutex);
      this->intermediate_outcome_map[state] =
          std::make_shared<std::string>(outcome);
    }));
  }

  // Wait for states to finish
  for (std::thread &state_thread : state_threads) {
    if (state_thread.joinable()) {
      state_thread.join();
    }
  }

  // Handle a cancel
  if (is_canceled()) {
    return default_outcome; // ? not sure waht else to return here
  }

  // Build final outcome
  std::set<std::string> satisfied_outcomes;
  for (const auto &[outcome, requirements] : outcome_map) {
    bool satisfied = true;
    for (const auto &[state, expected_intermediate_outcome] : requirements) {
      std::shared_ptr<std::string> actual_intermediate_outcome =
          intermediate_outcome_map.find(state)->second;
      if (actual_intermediate_outcome == nullptr) {
        throw std::runtime_error("An intermediate outcome for state '" +
                                 state->to_string() + "' was not received.");
      }
      satisfied &=
          *actual_intermediate_outcome == expected_intermediate_outcome;
    }
    if (satisfied) {
      satisfied_outcomes.insert(outcome);
    }
  }

  // Handle different numbers of satisfied outcomes
  if (satisfied_outcomes.size() == 0) {
    return default_outcome;
  } else if (satisfied_outcomes.size() > 1) {
    std::string outcomes_string;
    for (auto it = satisfied_outcomes.begin(); it != satisfied_outcomes.end();
         ++it) {
      outcomes_string += *it;

      // Add a comma if this is not the last element
      if (std::next(it) != outcomes.end()) {
        outcomes_string += ", ";
      }
    }
    // Due to how std::set works, this should only throw if the outcome strings
    // are disaprate
    throw std::logic_error(
        "More than one satisfied outcomes (" + outcomes_string +
        ") after concurrent state execution (" + this->to_string());
  }

  return *satisfied_outcomes.begin();
}

void Concurrence::cancel_state() {
  for (auto state : states) {
    state->cancel_state();
  }
  yasmin::State::cancel_state();
}

std::set<std::string>
Concurrence::generate_possible_outcomes(const OutcomeMap &outcome_map,
                                        const std::string &default_outcome) {
  std::set<std::string> possible_outcomes;
  possible_outcomes.insert(
      default_outcome); // Always include the default outcome

  for (const auto &[outcome, requirements] : outcome_map) {
    possible_outcomes.insert(outcome);
  }

  return possible_outcomes;
}