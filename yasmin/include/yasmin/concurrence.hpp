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
  typedef std::map<std::string, std::string> StateOutcomeMap;
  typedef std::map<std::string, StateOutcomeMap> OutcomeMap;

protected:
  /// The states to run concurrently (name -> state)
  const std::map<std::string, std::shared_ptr<State>> states;

  /// Default outcome
  const std::string default_outcome;

  /// Specifies which combination of state outputs should produce a given
  /// overall output
  OutcomeMap outcome_map;

  /// Stores the intermediate outcomes of the concurrent states
  std::map<std::string, std::shared_ptr<std::string>> intermediate_outcome_map;

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
   * @param states A map of state names to states that will run concurrently.
   * @param default_outcome The default outcome to return if no outcome map
   * rules are satisfied.
   * @param outcome_map A map of outcome names to requirements for achieving
   * that outcome.
   */
  Concurrence(const std::map<std::string, std::shared_ptr<State>> &states,
              const std::string &default_outcome,
              const OutcomeMap &outcome_map);

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
   * @brief Returns the map of states managed by this concurrence state.
   * @return A map of state names to states.
   */
  const std::map<std::string, std::shared_ptr<State>> &get_states() const;

  /**
   * @brief Returns the outcome map for this concurrence state.
   * @return A map of outcome names to their requirements.
   */
  const OutcomeMap &get_outcome_map() const;

  /**
   * @brief Returns the default outcome for this concurrence state.
   * @return The default outcome as a string.
   */
  const std::string &get_default_outcome() const;

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   */
  std::string to_string() override {
    std::string name = "Concurrence [";

    for (auto it = states.begin(); it != states.end(); ++it) {
      name += it->first + " (" + it->second->to_string() + ")";

      // Add a comma if this is not the last element
      if (std::next(it) != states.end()) {
        name += ", ";
      }
    }

    name += "]";

    return name;
  }
};

} // namespace yasmin

#endif // YASMIN__CONCURRENCE_HPP
