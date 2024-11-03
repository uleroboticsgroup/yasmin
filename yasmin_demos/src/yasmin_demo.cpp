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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
class FooState : public yasmin::State {
public:
  /// Counter to track the number of executions.
  int counter;

  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState() : yasmin::State({"outcome1", "outcome2"}), counter(0){};

  /**
   * @brief Executes the Foo state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * increments the counter, and sets a string in the blackboard.
   * The state will transition to either "outcome1" or "outcome2"
   * based on the current value of the counter.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state FOO");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (this->counter < 3) {
      this->counter += 1;
      blackboard->set<std::string>("foo_str",
                                   "Counter: " + std::to_string(this->counter));
      return "outcome1";

    } else {
      return "outcome2";
    }
  };
};

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
  BarState() : yasmin::State({"outcome3"}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * retrieves a string from the blackboard, and logs it.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

    return "outcome3";
  }
};

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("FOO", std::make_shared<FooState>(),
                {
                    {"outcome1", "BAR"},
                    {"outcome2", "outcome4"},
                });
  sm->add_state("BAR", std::make_shared<BarState>(),
                {
                    {"outcome3", "FOO"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("yasmin_demo", sm);

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
