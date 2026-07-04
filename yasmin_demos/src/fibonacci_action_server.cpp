// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @class FibonacciActionServer
 * @brief A ROS 2 action server node for calculating Fibonacci sequences.
 */
class FibonacciActionServer : public rclcpp::Node {
public:
  /**
   * @brief Alias for the Fibonacci action type.
   */
  using Fibonacci = example_interfaces::action::Fibonacci;

  /**
   * @brief Alias for the goal handle of the Fibonacci action.
   */
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  /**
   * @brief Constructor for the FibonacciActionServer.
   *
   * Initializes the action server and sets up the goal, cancel, and accepted
   * callbacks.
   * @param options Node options for initialization.
   */
  explicit FibonacciActionServer() : Node("fibonacci_action_server") {
    this->threads_.reserve(8);

    // Callback to handle goal requests.
    auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const Fibonacci::Goal> goal) {
      (void)uuid;
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
                  goal->order);
      if (goal->order > 46) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // Callback to handle goal cancellation requests.
    auto handle_cancel =
        [this](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    // Callback to handle accepted goals.
    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
          auto execute_in_thread = [this, goal_handle]() {
            return this->execute(goal_handle);
          };
          {
            std::lock_guard<std::mutex> lock(this->threads_mutex_);
            this->threads_.emplace_back(execute_in_thread);
          }
        };

    // Create the Fibonacci action server.
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci", handle_goal, handle_cancel, handle_accepted);

    RCLCPP_INFO(this->get_logger(), "Fibonacci Server started");
  }

  ~FibonacciActionServer() {
    for (auto &thread : this->threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

private:
  /**
   * @brief Mutex protecting the threads vector.
   */
  std::mutex threads_mutex_;
  /**
   * @brief Tracked execution threads, joined on destruction.
   */
  std::vector<std::thread> threads_;

  /**
   * @brief The action server instance for Fibonacci calculations.
   */
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  /**
   * @brief Executes the Fibonacci calculation for a given goal.
   *
   * Generates the Fibonacci sequence up to the requested order, providing
   * feedback to the client and handling cancellation requests.
   * @param goal_handle Shared pointer to the goal handle.
   */
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate rate(1);

    // Retrieve the goal details.
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    // Initialize the Fibonacci sequence with the first two numbers.
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<Fibonacci::Result>();

    if (goal->order < 0) {
      RCLCPP_ERROR(this->get_logger(), "Goal order must be non-negative");
      goal_handle->abort(result);
      return;
    }

    // Generate the Fibonacci sequence up to the requested order.
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {

      // Check if there is a cancel request.
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Update the sequence with the next number.
      sequence.push_back(sequence[i] + sequence[i - 1]);

      // Publish feedback to the client.
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      rate.sleep();
    }

    // Complete the goal if the process is not canceled.
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class FibonacciActionServer

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
