// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__JOIN_STATE_HPP_
#define YASMIN__JOIN_STATE_HPP_

#include <string>

#include "yasmin/region_barrier.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

/**
 * @class JoinState
 * @brief Represents a state that waits for multiple concurrent states to
 * complete.
 *
 * The JoinState class is a specialized state that synchronizes the execution of
 * multiple concurrent states. It waits for all specified states to reach a
 * synchronization point before proceeding to the next state. This is useful in
 * scenarios where multiple parallel tasks need to be completed before moving
 * forward.
 */
class JoinState : public State {
public:
  /** @brief Shared pointer aliases for JoinState. */
  YASMIN_PTR_ALIASES(JoinState)

  /**
   * @brief Constructs a new JoinState instance.
   */
  JoinState();

  /**
   * @brief Constructs a new JoinState instance with a specified synchronization
   * ID and an optional outcome.
   * @param sync_id The unique identifier for the synchronization point.
   * @param outcome The outcome to return when the join is complete (default is
   * "joined").
   */
  JoinState(const std::string &sync_id, const std::string &outcome = "joined");

  /**
   * @brief Configures the JoinState.
   */
  void configure() override;

  /**
   * @brief Sets the barrier for the JoinState.
   * @param barrier A shared pointer to the RegionBarrier to use for
   * synchronization.
   */
  void set_barrier(RegionBarrier::SharedPtr barrier);

  /**
   * @brief Gets the synchronization ID for the JoinState.
   * @return The synchronization ID as a string.
   */
  const std::string &get_sync_id() const;

  /**
   * @brief Gets the barrier for the JoinState.
   * @return A shared pointer to the RegionBarrier used for synchronization.
   */
  RegionBarrier::SharedPtr get_barrier() const;

  /**
   * @brief Executes the JoinState.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method waits for all concurrent states to reach the synchronization
   * point defined by sync_id_. Once all states have synchronized, it returns
   * the specified outcome.
   */
  std::string execute(Blackboard::SharedPtr blackboard) override;

  /**
   * @brief Converts the JoinState to a string representation.
   * @return A string representation of the JoinState.
   *
   * This method retrieves the demangled name of the class for a readable
   * string representation.
   */
  std::string to_string() const override;

private:
  /// @brief The unique identifier for the synchronization point.
  std::string sync_id_;
  /// @brief A shared pointer to the RegionBarrier used for synchronization.
  RegionBarrier::SharedPtr barrier_;
  /// @brief The outcome to return when the join is complete.
  std::string outcome_;
};

} // namespace yasmin

#endif // YASMIN__JOIN_STATE_HPP_
