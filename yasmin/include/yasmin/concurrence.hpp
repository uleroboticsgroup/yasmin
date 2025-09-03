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

#ifndef YASMIN__CONCURRENCE_HPP
#define YASMIN__CONCURRENCE_HPP

#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifdef __GNUG__     // If using GCC/G++
#include <cxxabi.h> // For abi::__cxa_demangle
#endif

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

/**
 * @class Concurrence
 * @brief Runs a series of states in parallel
 *
 * The Concurrence class runs a set of states concurrently, waiting
 * for the termination of each, and then returns a single output
 * according to a provided rule map, or a default outcome if no rule is
 * satisfied.
 */
class Concurrence : public State {

public:
  typedef std::map<std::shared_ptr<State>, std::string> StateMap;
  typedef std::map<std::string, StateMap> OutcomeMap;

protected:
  /// The states to run concurrently
  const std::set<std::shared_ptr<State>> states;

  /// Default outcome
  const std::string default_outcome;

  /// Specifies which combination of state outputs should produce a given
  /// overall output
  OutcomeMap outcome_map;

  /// Stores the intermedaite outcomes of the concurrent states
  std::map<std::shared_ptr<State>, std::shared_ptr<std::string>>
      intermediate_outcome_map;

  /// The set of possible outcomes
  std::set<std::string> possible_outcomes;

private:
  /// Mutex for intermediate outcome map
  std::mutex intermediate_outcome_mutex;

  /// @brief Helper function to generate a set of possible outcomes from an
  /// outcome map
  /// @param outcome_map
  /// @param default_outcome
  /// @return The set of possible outcomes
  static std::set<std::string>
  generate_possible_outcomes(const OutcomeMap &outcome_map,
                             const std::string &default_outcome);

public:
  /**
   * @brief Constructs a State with a set of possible outcomes.
   * @param outcomes A set of possible outcomes for this state.
   */
  Concurrence(std::set<std::shared_ptr<State>> states,
              std::string default_outcome, OutcomeMap outcome_map);

  /**
   * @brief Executes the state's specific logic.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method is intended to be overridden by derived classes to provide
   * specific execution logic.
   */
  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;

  /**
   * @brief Cancels the current state execution.
   *
   * This method sets the canceled flag to true and logs the action.
   */
  void cancel_state() override;

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   *
   * This method retrieves the demangled name of the class for a readable
   * string representation.
   */
  std::string to_string() override {
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

    name += "[";

    for (auto it = states.begin(); it != states.end(); ++it) {
      name += (*it)->to_string();

      // Add a comma if this is not the last element
      if (std::next(it) != states.end()) {
        name += ", ";
      }
    }

    name += "]";

    return name; // Return the demangled class name
  }
};

} // namespace yasmin

#endif // YASMIN__CONCURRENCE_HPP
