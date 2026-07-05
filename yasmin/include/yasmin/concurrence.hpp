// Copyright (C) 2025 Georgia Tech Research Institute
// Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YASMIN__CONCURRENCE_HPP_
#define YASMIN__CONCURRENCE_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

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

protected:
  /// The states to run concurrently (name -> state)
  const StateMap states;

  /// Default outcome
  const std::string default_outcome;

  /// Specifies which combination of state outputs should produce a given
  /// overall output
  OutcomeMap outcome_map;

  /// Stores the intermediate outcomes of the concurrent states
  StateOutcomeMap intermediate_outcome_map;

  /// The set of possible outcomes
  Outcomes possible_outcomes;
  /// Per-child parameter mappings applied during configure()
  ParameterMappingsMap parameter_mappings;

private:
  /// Mutex for intermediate outcome map
  std::mutex intermediate_outcome_mutex;

  /// Flag to indicate if this concurrence state has been configured
  std::atomic_bool configured{false};

  /**
   * @brief Applies this container's parameter mappings to a direct child.
   * @param state_name The child state name.
   * @param state The child state instance.
   */
  void apply_parameter_mappings(const std::string &state_name,
                                const State::SharedPtr &state);

public:
  /**
   * @brief Shared pointer type for Concurrence.
   */
  YASMIN_PTR_ALIASES(Concurrence)

  /**
   * @brief Constructs a Concurrence with states running in parallel.
   * @param states A map of state names to states that will run concurrently.
   * @param default_outcome The default outcome to return if no outcome map
   * rules are satisfied.
   * @param outcome_map A map of outcome names to requirements for achieving
   * that outcome.
   * @param parameter_mappings Per-child parameter mappings.
   */
  Concurrence(const StateMap &states, const std::string &default_outcome,
              const OutcomeMap &outcome_map,
              const ParameterMappingsMap &parameter_mappings = {});

  /**
   * @brief Sets parameter mappings for a child state.
   * @param state_name The child state name.
   * @param parameter_mappings Mapping entries child_parameter ->
   * parent_parameter.
   */
  void set_parameter_mappings(const std::string &state_name,
                              const ParameterMappings &parameter_mappings);

  /**
   * @brief Returns the parameter mappings for this concurrence state.
   * @return A constant reference to the parameter mappings.
   */
  const ParameterMappingsMap &get_parameter_mappings() const noexcept;

  /**
   * @brief Configures this concurrence state and all of its children.
   */
  void configure() override;

  /**
   * @brief Executes the state's specific logic.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method is intended to be overridden by derived classes to provide
   * specific execution logic.
   */
  std::string execute(Blackboard::SharedPtr blackboard) override;

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
  const StateMap &get_states() const noexcept;

  /**
   * @brief Returns the outcome map for this concurrence state.
   * @return A map of outcome names to their requirements.
   */
  const OutcomeMap &get_outcome_map() const noexcept;

  /**
   * @brief Returns the default outcome for this concurrence state.
   * @return The default outcome as a string.
   */
  const std::string &get_default_outcome() const noexcept;

  /**
   * @brief Validates the nested states of this concurrence.
   * @param strict_mode If true, performs strict validation.
   *
   * Recursively validates any StateMachine, Concurrence, or OrthogonalState
   * instances found in the concurrent states map.
   */
  void validate(bool strict_mode = false);

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   */
  std::string to_string() const override;
};

} // namespace yasmin

#endif // YASMIN__CONCURRENCE_HPP_
