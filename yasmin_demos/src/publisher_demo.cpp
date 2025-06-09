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

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

/**
 * @class PublishIntState
 * @brief A state that publishes ints.
 *
 * This class inherits from yasmin_ros::PublisherState and publish ints
 * to the "count" topic.
 */
class PublishIntState
    : public yasmin_ros::PublisherState<std_msgs::msg::Int32> {

public:
  /// Counter to keep track of how many times the message has been published
  int counter;

  /**
   * @brief Constructor for the PublishIntState class.
   */
  PublishIntState()
      : yasmin_ros::PublisherState<std_msgs::msg::Int32>(
            "count", // topic name
            std::bind(&PublishIntState::create_int_msg, this,
                      _1) // create msg handler callback
        ) {
    this->counter = 0;
  };

  /**
   * @brief Create a new Int message.
   *
   *
   * @param blackboard Shared pointer to the blackboard (unused in this
   * implementation).
   * @return A new Int message.
   */
  std_msgs::msg::Int32
  create_int_msg(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    (void)blackboard; // blackboard is not used in this implementation

    this->counter++;
    YASMIN_LOG_INFO("Creating message %d", this->counter);
    std_msgs::msg::Int32 msg;
    msg.data = this->counter;
    return msg;
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
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("PUBLISHING_INT", std::make_shared<PublishIntState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     "PUBLISHING_INT"}, // Transition back to itself
                });

  // Publisher for visualizing the state machine's status
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_PUBLISHER_DEMO", sm);

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
