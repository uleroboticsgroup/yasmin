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

#include <iostream>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

/**
 * @class PrintOdometryState
 * @brief A state that monitors odometry data and transitions based on a
 * specified count.
 *
 * This class inherits from yasmin_ros::MonitorState and listens to the "odom"
 * topic for nav_msgs::msg::Odometry messages. The state transitions once a
 * specified number of messages has been received and processed.
 */
class PrintOdometryState
    : public yasmin_ros::MonitorState<nav_msgs::msg::Odometry> {

public:
  /// The number of times the state will process messages
  int times;

  /**
   * @brief Constructor for the PrintOdometryState class.
   * @param times Number of times to print odometry data before transitioning.
   */
  PrintOdometryState(int times)
      : yasmin_ros::MonitorState<nav_msgs::msg::Odometry>(
            "odom",                   // topic name
            {"outcome1", "outcome2"}, // possible outcomes
            std::bind(&PrintOdometryState::monitor_handler, this, _1,
                      _2), // monitor handler callback
            10,            // QoS for the topic subscription
            10,            // queue size for the callback
            10             // timeout for receiving messages
        ) {
    this->times = times;
  };

  /**
   * @brief Handler for processing odometry data.
   *
   * This function logs the x, y, and z positions from the odometry message.
   * After processing, it decreases the `times` counter. When the counter
   * reaches zero, the state transitions to "outcome2"; otherwise, it remains in
   * "outcome1".
   *
   * @param blackboard Shared pointer to the blackboard (unused in this
   * implementation).
   * @param msg Shared pointer to the received odometry message.
   * @return A string representing the outcome: "outcome1" to stay in the state,
   *         or "outcome2" to transition out of the state.
   */
  std::string
  monitor_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                  std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    (void)blackboard; // blackboard is not used in this implementation

    YASMIN_LOG_INFO("x: %f", msg->pose.pose.position.x);
    YASMIN_LOG_INFO("y: %f", msg->pose.pose.position.y);
    YASMIN_LOG_INFO("z: %f", msg->pose.pose.position.z);

    this->times--;

    // Transition based on remaining times
    if (this->times <= 0) {
      return "outcome2";
    }

    return "outcome1";
  };
};

/**
 * @brief Main function initializing ROS 2 and setting up the state machine.
 *
 * Initializes ROS 2, configures loggers, sets up the state machine with states
 * and transitions, and starts monitoring odometry data. The state machine will
 * cancel upon ROS 2 shutdown.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 *
 * @exception std::exception Catches and logs any exceptions thrown by the state
 * machine.
 */
int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_monitor_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  // Create a state machine with a final outcome
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state(
      "PRINTING_ODOM", std::make_shared<PrintOdometryState>(5),
      {
          {"outcome1",
           "PRINTING_ODOM"},        // Transition back to itself on outcome1
          {"outcome2", "outcome4"}, // Transition to outcome4 on outcome2
          {yasmin_ros::basic_outcomes::TIMEOUT,
           "outcome4"}, // Timeout transition
      });

  // Publisher for visualizing the state machine's status
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_MONITOR_DEMO", sm);

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
