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

#include "yasmin/cb_state.hpp"
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
  /**
   * @brief Constructor for the PublishIntState class.
   */
  PublishIntState()
      : yasmin_ros::PublisherState<std_msgs::msg::Int32>(
            "count", // topic name
            std::bind(&PublishIntState::create_int_msg, this,
                      _1) // create msg handler callback
        ){};

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

    int counter = blackboard->get<int>("counter");
    counter++;
    blackboard->set<int>("counter", counter);

    YASMIN_LOG_INFO("Creating message %d", counter);
    std_msgs::msg::Int32 msg;
    msg.data = counter;
    return msg;
  };
};

/**
 * @brief Check the count in the blackboard and return an outcome based on it.
 *
 * This function checks the value of "counter" in the blackboard and compares it
 * with "max_count". If "counter" exceeds "max_count", it returns "outcome1",
 * otherwise it returns "outcome2".
 *
 * @param blackboard Shared pointer to the blackboard.
 * @return A string representing the outcome.
 */
std::string
check_count(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  // Sleep for 1 second to simulate some processing time
  rclcpp::sleep_for(std::chrono::seconds(1));
  YASMIN_LOG_INFO("Checking count: %d", blackboard->get<int>("counter"));

  if (blackboard->get<int>("counter") >= blackboard->get<int>("max_count")) {
    return "outcome1";
  } else {
    return "outcome2";
  }
}

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

  YASMIN_LOG_INFO("yasmin_publisher_demo");
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
                     "CHECKINNG_COUNTS"}, // Transition back to itself
                });
  sm->add_state("CHECKINNG_COUNTS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{"outcome1", "outcome2"},
                    check_count),
                {{"outcome1", yasmin_ros::basic_outcomes::SUCCEED},
                 {"outcome2", "PUBLISHING_INT"}});

  // Publisher for visualizing the state machine's status
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_PUBLISHER_DEMO", sm);

  // Execute the state machine
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("counter", 0);
  blackboard->set<int>("max_count", 10);
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
