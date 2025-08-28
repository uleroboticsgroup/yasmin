// Copyright (C) 2025 Miguel Ángel González Santamarta
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
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 */
class FooState : public yasmin::State {
public:
  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}){};

  /**
   * @brief Executes the Foo state logic.
   *
   * Executes the logic for the Foo state.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution, which can be SUCCEED.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    std::string data = blackboard->get<std::string>("foo_data");
    YASMIN_LOG_INFO("%s", data.c_str());
    blackboard->set<std::string>("foo_out_data", data);
    return yasmin_ros::basic_outcomes::SUCCEED;
  };
};

/**
 * @brief Represents the "Bar" state in the state machine.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * Executes the logic for the Bar state.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    std::string datga = blackboard->get<std::string>("bar_data");
    YASMIN_LOG_INFO("%s", datga.c_str());
    return yasmin_ros::basic_outcomes::SUCCEED;
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
  YASMIN_LOG_INFO("yasmin_remapping_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create blackboard
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<std::string>("msg1", "test1");
  blackboard->set<std::string>("msg2", "test2");

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("STATE1", std::make_shared<FooState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "STATE2"},
                },
                {
                    {"foo_data", "msg1"},
                });
  sm->add_state("STATE2", std::make_shared<FooState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "STATE3"},
                },
                {
                    {"foo_data", "msg2"},
                });
  sm->add_state("STATE3", std::make_shared<BarState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::SUCCEED},
                },
                {
                    {"bar_data", "foo_out_data"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_REMAPPING_DEMO", sm);

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
