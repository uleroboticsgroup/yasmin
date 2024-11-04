// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS_ACTION_STATE_HPP
#define YASMIN_ROS_ACTION_STATE_HPP

#include <condition_variable>
#include <functional>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::placeholders;

namespace yasmin_ros {

/**
 * @brief A state class for handling ROS 2 action client operations.
 *
 * This class encapsulates the behavior of a ROS 2 action client within a YASMIN
 * state. It allows the creation and management of goals, feedback, and results
 * associated with an action server.
 *
 * @tparam ActionT The type of the action this state will interface with.
 */
template <typename ActionT> class ActionState : public yasmin::State {
  /// Alias for the action goal type.
  using Goal = typename ActionT::Goal;
  /// Alias for the action result type.
  using Result = typename ActionT::Result::SharedPtr;

  /// Alias for the action feedback type.
  using Feedback = typename ActionT::Feedback;
  /// Options for sending goals.
  using SendGoalOptions =
      typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  /// Shared pointer type for the action client.
  using ActionClient = typename rclcpp_action::Client<ActionT>::SharedPtr;
  /// Handle for the action goal.
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  /// Function type for creating a goal.
  using CreateGoalHandler =
      std::function<Goal(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  /// Function type for handling results.
  using ResultHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Result)>;
  /// Function type for handling feedback.
  using FeedbackHandler =
      std::function<void(std::shared_ptr<yasmin::blackboard::Blackboard>,
                         std::shared_ptr<const Feedback>)>;

public:
  /**
   * @brief Construct an ActionState with a specific action name and goal
   * handler.
   *
   * This constructor initializes the action state with a specified action name,
   * goal handler, and optional timeout.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param timeout (Optional) The maximum time to wait for the action server.
   *                Default is -1 (no timeout).
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              std::set<std::string> outcomes, int timeout = -1.0)
      : ActionState(action_name, create_goal_handler, outcomes, nullptr,
                    nullptr, timeout) {}

  /**
   * @brief Construct an ActionState with result and feedback handlers.
   *
   * This constructor allows the specification of result and feedback handlers.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param timeout (Optional) The maximum time to wait for the action server.
   *                Default is -1 (no timeout).
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr, int timeout = -1.0)
      : ActionState(action_name, create_goal_handler, {}, result_handler,
                    feedback_handler, timeout) {}

  /**
   * @brief Construct an ActionState with outcomes and handlers.
   *
   * This constructor allows specifying outcomes along with handlers for results
   * and feedback.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param timeout (Optional) The maximum time to wait for the action server.
   *                Default is -1 (no timeout).
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              std::set<std::string> outcomes,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr, int timeout = -1.0)
      : ActionState(nullptr, action_name, create_goal_handler, outcomes,
                    result_handler, feedback_handler, timeout) {}

  /**
   * @brief Construct an ActionState with a specified node and action name.
   *
   * This constructor allows specifying a ROS 2 node in addition to the action
   * name and goal handler.
   *
   * @param node A shared pointer to the ROS 2 node.
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param timeout (Optional) The maximum time to wait for the action server.
   *                Default is -1 (no timeout).
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const rclcpp::Node::SharedPtr &node, std::string action_name,
              CreateGoalHandler create_goal_handler,
              std::set<std::string> outcomes,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr, int timeout = -1.0)
      : State({}), action_name(action_name),
        create_goal_handler(create_goal_handler),
        result_handler(result_handler), feedback_handler(feedback_handler),
        timeout(timeout) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT,
                      basic_outcomes::CANCEL};

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.insert(outcome);
      }
    }

    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    this->action_client =
        rclcpp_action::create_client<ActionT>(this->node_, action_name);

    if (this->create_goal_handler == nullptr) {
      throw std::invalid_argument("create_goal_handler is needed");
    }
  }

  /**
   * @brief Cancel the current action state.
   *
   * This function cancels the ongoing action and waits for the cancellation to
   * complete.
   */
  void cancel_state() {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);

    if (this->goal_handle) {
      this->action_client->async_cancel_goal(
          this->goal_handle, std::bind(&ActionState::cancel_done, this));

      std::unique_lock<std::mutex> lock(this->action_cancel_mutex);
      this->action_cancel_cond.wait(lock);
    }

    yasmin::State::cancel_state();
  }

  /**
   * @brief Notify that the action cancellation has completed.
   *
   * This function is called to notify that the action cancellation process
   * has finished.
   */
  void cancel_done() { this->action_cancel_cond.notify_all(); }

  /**
   * @brief Execute the action and return the outcome.
   *
   * This function creates a goal using the provided goal handler, sends the
   * goal to the action server, and waits for the result or feedback.
   *
   * @param blackboard A shared pointer to the blackboard used for
   * communication.
   * @return A string representing the outcome of the action execution.
   *
   * Possible outcomes include:
   * - `basic_outcomes::SUCCEED`: The action succeeded.
   * - `basic_outcomes::ABORT`: The action was aborted.
   * - `basic_outcomes::CANCEL`: The action was canceled.
   * - `basic_outcomes::TIMEOUT`: The action server was not available in time.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    std::unique_lock<std::mutex> lock(this->action_done_mutex);

    Goal goal = this->create_goal_handler(blackboard);

    // Wait for the action server to be available
    RCLCPP_INFO(this->node_->get_logger(), "Waiting for action '%s'",
                this->action_name.c_str());
    bool act_available = this->action_client->wait_for_action_server(
        std::chrono::duration<int64_t, std::ratio<1>>(this->timeout));

    if (!act_available) {
      RCLCPP_WARN(this->node_->get_logger(),
                  "Timeout reached, action '%s' is not available",
                  this->action_name.c_str());
      return basic_outcomes::TIMEOUT;
    }

    // Prepare options for sending the goal
    SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
        std::bind(&ActionState::goal_response_callback, this, _1);

    send_goal_options.result_callback =
        std::bind(&ActionState::result_callback, this, _1);

    if (this->feedback_handler) {
      send_goal_options.feedback_callback =
          [this, blackboard](typename GoalHandle::SharedPtr,
                             std::shared_ptr<const Feedback> feedback) {
            this->feedback_handler(blackboard, feedback);
          };
    }

    RCLCPP_INFO(this->node_->get_logger(), "Sending goal to action '%s'",
                this->action_name.c_str());
    this->action_client->async_send_goal(goal, send_goal_options);

    // Wait for the action to complete
    this->action_done_cond.wait(lock);

    switch (this->action_status) {
    case rclcpp_action::ResultCode::CANCELED:
      return basic_outcomes::CANCEL;

    case rclcpp_action::ResultCode::ABORTED:
      return basic_outcomes::ABORT;

    case rclcpp_action::ResultCode::SUCCEEDED:
      if (this->result_handler) {
        return this->result_handler(blackboard, this->action_result);
      }
      return basic_outcomes::SUCCEED;

    default:
      return basic_outcomes::ABORT;
    }
  }

private:
  /// Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;

  /// Name of the action to communicate with.
  std::string action_name;

  /// Shared pointer to the action client.
  ActionClient action_client;
  /// Condition variable for action completion.
  std::condition_variable action_done_cond;

  /// Mutex for protecting action completion.
  std::mutex action_done_mutex;
  /// Condition variable for action cancellation.
  std::condition_variable action_cancel_cond;
  /// Mutex for protecting action cancellation.
  std::mutex action_cancel_mutex;

  /// Shared pointer to the action result.
  Result action_result;
  /// Status of the action execution.
  rclcpp_action::ResultCode action_status;

  /// Handle for the current goal.
  std::shared_ptr<GoalHandle> goal_handle;
  /// Mutex for protecting access to the goal handle.
  std::mutex goal_handle_mutex;

  /// Handler function for creating goals.
  CreateGoalHandler create_goal_handler;
  /// Handler function for processing results.
  ResultHandler result_handler;
  /// Handler function for processing feedback.
  FeedbackHandler feedback_handler;

  /// Maximum time to wait for the action server.
  int timeout;

#if defined(FOXY)
  /**
   * @brief Callback for handling the goal response.
   *
   * This function is called when a response for the goal is received.
   *
   * @param future A future that holds the goal handle.
   */
  void goal_response_callback(
      std::shared_future<typename GoalHandle::SharedPtr> future) {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    this->goal_handle = future.get();
  }
#else
  /**
   * @brief Callback for handling the goal response.
   *
   * This function is called when a response for the goal is received.
   *
   * @param goal_handle A shared pointer to the goal handle.
   */
  void
  goal_response_callback(const typename GoalHandle::SharedPtr &goal_handle) {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    this->goal_handle = goal_handle;
  }
#endif

  /**
   * @brief Callback for handling the result of the action.
   *
   * This function is called when the action result is available.
   *
   * @param result The wrapped result of the action.
   */
  void result_callback(const typename GoalHandle::WrappedResult &result) {
    this->action_result = result.result;
    this->action_status = result.code;
    this->action_done_cond.notify_one();
  }
};

} // namespace yasmin_ros

#endif // YASMIN_ROS_ACTION_STATE_HPP
