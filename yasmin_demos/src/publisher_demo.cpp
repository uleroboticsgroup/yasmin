// Copyright (C) 2025 Miguel Ángel González Santamarta
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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;

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
        ) {
    this->set_description("Publishes an incrementing integer to the 'count' "
                          "topic using values stored in the blackboard.");
    this->add_input_key<int>(
        "counter", "Current counter value stored in the blackboard.", 0);
    this->add_output_key("counter",
                         "Updated counter value after incrementing.");
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
  create_int_msg(yasmin::Blackboard::SharedPtr blackboard) {

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
std::string check_count(yasmin::Blackboard::SharedPtr blackboard) {

  // Sleep for 1 second to simulate some processing time
  rclcpp::sleep_for(std::chrono::seconds(1));
  YASMIN_LOG_INFO("Checking count: %d", blackboard->get<int>("counter"));

  if (blackboard->get<int>("counter") >= blackboard->get<int>("max_count")) {
    return "outcome1";
  } else {
    return "outcome2";
  }
}

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();
  YASMIN_LOG_INFO("yasmin_publisher_demo");

  // Create a state machine with a final outcome
  auto sm = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED},
      true);
  sm->set_description("Publishes incrementing integers until the configured "
                      "maximum count is reached.");
  sm->add_input_key<int>("counter",
                         "Current counter value stored in the blackboard.", 0);
  sm->add_input_key<int>(
      "max_count", "Maximum counter threshold used to stop publishing.", 10);
  sm->add_output_key("counter",
                     "Updated counter value after each publish step.");

  auto checking_counts_state = yasmin::CbState::make_shared(
      std::initializer_list<std::string>{"outcome1", "outcome2"}, check_count);
  checking_counts_state->set_description(
      "Checks whether the counter stored in the blackboard reached the "
      "configured maximum.");
  checking_counts_state->add_input_key<int>("counter", "Current counter value.",
                                            0);
  checking_counts_state->add_input_key<int>(
      "max_count", "Maximum counter threshold used to stop publishing.", 10);

  // Add states to the state machine
  sm->add_state("PUBLISHING_INT", std::make_shared<PublishIntState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     "CHECKING_COUNTS"}, // Transition back to itself
                });
  sm->add_state("CHECKING_COUNTS", checking_counts_state,
                {{"outcome1", yasmin_ros::basic_outcomes::SUCCEED},
                 {"outcome2", "PUBLISHING_INT"}});

  yasmin::Blackboard::SharedPtr blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<int>("counter", 0);
  blackboard->set<int>("max_count", 10);

  {
    // Publisher for visualizing the state machine's status
    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_PUBLISHER_DEMO");

    // Execute the state machine
    try {
      std::string outcome = (*sm.get())(blackboard);
      YASMIN_LOG_INFO(outcome.c_str());
    } catch (const std::exception &e) {
      YASMIN_LOG_WARN(e.what());
    }
  }

  yasmin_ros::YasminNode::destroy_instance();
  rclcpp::shutdown();

  return 0;
}
