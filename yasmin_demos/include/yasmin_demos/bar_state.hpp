// Copyright (C) 2025 Pedro Edom Nunes
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

#ifndef YASMIN_DEMOS_BAR_STATE_HPP_
#define YASMIN_DEMOS_BAR_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState();

  /**
   * @brief Destructs the BarState object.
   */
  ~BarState() override = default;

  /**
   * @brief Configures the state-local parameters.
   */
  void configure() override;

  /**
   * @brief Executes the Bar state logic.
   *
   * This method logs the execution, waits for the configured duration,
   * retrieves a string from the blackboard, and logs it.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard);

private:
  /// @brief Prefix for log messages
  std::string log_prefix_;
  /// @brief Sleep duration in milliseconds for each execution
  int sleep_ms_;
};

#endif // YASMIN_DEMOS_BAR_STATE_HPP_
