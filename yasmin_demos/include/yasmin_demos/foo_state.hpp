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

#ifndef YASMIN_DEMOS_FOO_STATE_HPP_
#define YASMIN_DEMOS_FOO_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
class FooState : public yasmin::State {
public:
  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState();

  /**
   * @brief Destructs the FooState object.
   */
  ~FooState() override = default;

  /**
   * @brief Configures the state-local parameters.
   */
  void configure() override;

  /**
   * @brief Executes the Foo state logic.
   *
   * This method logs the execution, waits for the configured duration,
   * increments the counter, and sets a string in the blackboard.
   * The state will transition to either "outcome1" or "outcome2"
   * based on the current value of the counter.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard);

  /// Counter to track the number of executions.
  int counter;

private:
  /// @brief Prefix for the counter output
  std::string counter_prefix_;
  /// @brief Maximum count before the state resets
  int max_count_;
  /// @brief Sleep duration in milliseconds between increments
  int sleep_ms_;
};

#endif // YASMIN_DEMOS_FOO_STATE_HPP_
