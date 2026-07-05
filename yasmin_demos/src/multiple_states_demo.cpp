// Copyright (C) 2025 Pedro Edom Nunes
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

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/bar_state.hpp"
#include "yasmin_demos/foo_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();
  YASMIN_LOG_INFO("yasmin_multiple_states_demo");

  // Create a state machine
  auto sm = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{"outcome4"}, true);
  sm->set_description(
      "Runs a simple loop between FooState and BarState demonstrating how "
      "states can be organized across multiple files.");
  sm->add_output_key(
      "foo_str",
      "Formatted counter string produced by FooState and read by BarState.");

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

  {
    // Publisher for visualizing the state machine
    yasmin_viewer::YasminViewerPub yasmin_pub(sm,
                                              "YASMIN_MULTIPLE_STATES_DEMO");

    // Execute the state machine
    try {
      std::string outcome = (*sm.get())();
      YASMIN_LOG_INFO(outcome.c_str());
    } catch (const std::exception &e) {
      YASMIN_LOG_WARN(e.what());
    }
  }

  yasmin_ros::YasminNode::destroy_instance();
  rclcpp::shutdown();

  return 0;
}
