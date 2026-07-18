// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN_FACTORY__YASMIN_FACTORY_ACTION_SERVER_HPP_
#define YASMIN_FACTORY__YASMIN_FACTORY_ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp_action/rclcpp_action.hpp>

#include "yasmin_msgs/action/run_state_machine.hpp"

namespace yasmin {
class StateMachine;
} // namespace yasmin

namespace yasmin_ros {
class YasminNode;
} // namespace yasmin_ros

namespace yasmin_factory {

class YasminFactory;

/**
 * @class YasminFactoryActionServer
 * @brief Runs XML-defined YASMIN state machines through a ROS 2 action.
 *
 * The server accepts one goal at a time. Each accepted goal creates a fresh
 * state machine from the requested XML file, publishes the current top-level
 * state as feedback, and returns the final state-machine outcome. An empty goal
 * path falls back to the `state_machine_file` node parameter.
 */
class YasminFactoryActionServer {
public:
  /// @brief Action type used to request state-machine execution.
  using RunStateMachine = yasmin_msgs::action::RunStateMachine;

  /// @brief Goal-handle type for RunStateMachine action goals.
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunStateMachine>;

  /**
   * @brief Constructs the action server on an existing YASMIN ROS node.
   * @param node Node used to host the action server and its parameters.
   */
  explicit YasminFactoryActionServer(
      const std::shared_ptr<yasmin_ros::YasminNode> &node);

  /**
   * @brief Stops any active execution and releases action-server resources.
   */
  ~YasminFactoryActionServer();

  /**
   * @brief Shuts down the server and waits for its execution thread to finish.
   *
   * Calling this method more than once has no effect.
   */
  void shutdown();

private:
  /**
   * @brief Accepts a goal when the server is idle and an XML path is available.
   * @param uuid Unique identifier assigned to the incoming goal.
   * @param goal Requested state-machine execution.
   * @return ACCEPT_AND_EXECUTE when the goal can run, otherwise REJECT.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const RunStateMachine::Goal> goal);

  /**
   * @brief Requests cancellation of the currently active state machine.
   * @param goal_handle Goal requesting cancellation.
   * @return ACCEPT when an execution is active, otherwise REJECT.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> &goal_handle);

  /**
   * @brief Starts execution of an accepted goal on the worker thread.
   * @param goal_handle Accepted goal to execute.
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  /**
   * @brief Publishes the current top-level state as action feedback.
   * @param goal_handle Goal receiving the feedback.
   * @param current_state Name of the current state.
   */
  void publish_feedback(const std::shared_ptr<GoalHandle> &goal_handle,
                        const std::string &current_state) const;

  /**
   * @brief Checks whether cancellation was requested for an execution.
   * @param goal_handle Goal whose cancellation state is checked.
   * @return True when cancellation was requested.
   */
  bool
  cancellation_requested(const std::shared_ptr<GoalHandle> &goal_handle) const;

  /// @brief Releases the active state machine and its factory.
  void clear_active_state_machine();

  /// @brief Marks the server as ready to accept another goal.
  void make_available();

  /**
   * @brief Loads and executes the state machine for an accepted goal.
   * @param goal_handle Goal containing the XML path and receiving the result.
   */
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  /// @brief Node hosting the action server.
  std::shared_ptr<yasmin_ros::YasminNode> node_;

  /// @brief Keeps the embedded factory runtime initialized while idle.
  std::unique_ptr<YasminFactory> runtime_initializer_;

  /// @brief ROS 2 action-server instance.
  rclcpp_action::Server<RunStateMachine>::SharedPtr action_server_;

  /// @brief XML path used when a goal does not provide one.
  std::string default_state_machine_file_;

  /// @brief Whether to publish the active state machine for the viewer.
  bool enable_viewer_pub_{true};

  /// @brief Protects execution and shutdown state shared by callbacks.
  mutable std::mutex state_mutex_;

  /// @brief Whether a goal currently owns the execution slot.
  bool busy_{false};

  /// @brief Whether cancellation was requested for the active goal.
  bool cancel_requested_{false};

  /// @brief Whether shutdown has started.
  bool shutting_down_{false};

  /// @brief Factory owning plugins used by the active state machine.
  std::shared_ptr<YasminFactory> active_factory_;

  /// @brief State machine currently executing.
  std::shared_ptr<yasmin::StateMachine> active_state_machine_;

  /// @brief Worker thread used to execute the active state machine.
  std::thread execution_thread_;
};

} // namespace yasmin_factory

#endif // YASMIN_FACTORY__YASMIN_FACTORY_ACTION_SERVER_HPP_
