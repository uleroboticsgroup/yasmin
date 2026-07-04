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

#ifndef YASMIN_DEMOS_WORKER_STATE_HPP_
#define YASMIN_DEMOS_WORKER_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

/**
 * @brief Represents a state that performs a simple counting task.
 *
 * This state increments a counter up to a specified maximum count, sleeping
 * for a defined duration between increments. It demonstrates basic state
 * functionality and interaction with the blackboard.
 */
class WorkerState : public yasmin::State {
public:
  /**
   * @brief Constructs a WorkerState object.
   */
  WorkerState();

  /**
   * @brief Destructs the WorkerState object.
   */
  ~WorkerState() override = default;

  /**
   * @brief Configures the state-local parameters.
   */
  void configure() override;

  /**
   * @brief Executes the Worker state logic.
   *
   * This method increments a counter up to a maximum count, sleeping for a
   * specified duration between increments. It demonstrates basic state
   * functionality and interaction with the blackboard.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard);

  /// @brief Internal counter tracking execution count.
  int counter;

private:
  /// @brief The maximum count value for the counter.
  int max_count_;
  /// @brief The sleep duration in milliseconds between counter increments.
  int sleep_ms_;
};

#endif // YASMIN_DEMOS_WORKER_STATE_H
