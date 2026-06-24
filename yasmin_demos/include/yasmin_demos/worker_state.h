// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

#ifndef YASMIN_DEMOS_WORKER_STATE_H
#define YASMIN_DEMOS_WORKER_STATE_H

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
  ~WorkerState();

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
  std::string execute(yasmin::Blackboard::SharedPtr /*blackboard*/);

  //// @brief Resets the internal counter to zero.
  int counter;

private:
  /// @brief The maximum count value for the counter.
  int max_count_;
  /// @brief The sleep duration in milliseconds between counter increments.
  int sleep_ms_;
};

#endif // YASMIN_DEMOS_WORKER_STATE_H
