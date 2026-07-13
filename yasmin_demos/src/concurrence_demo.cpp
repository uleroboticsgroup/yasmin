// Copyright (C) 2025 Georgia Tech Research Institute
// Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
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

#include "yasmin/concurrence.hpp"
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
  YASMIN_LOG_INFO("yasmin_concurrence_demo");

  // Create a state machine
  auto sm = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{"outcome4"}, true);
  sm->set_description(
      "Runs FooState and BarState concurrently until the concurrence state no "
      "longer matches the configured outcome map.");
  sm->add_output_key(
      "foo_str", "String containing the current counter value produced during "
                 "the concurrent execution.");

  // Create states to run concurrently
  auto foo_state = std::make_shared<FooState>();
  auto bar_state = std::make_shared<BarState>();

  // Create concurrent state
  auto concurrent_state = yasmin::Concurrence::make_shared(
      yasmin::StateMap{
          {"FOO", foo_state},
          {"BAR", bar_state},
      },
      "defaulted",
      yasmin::OutcomeMap{
          {"outcome1",
           {
               {"FOO", "outcome1"},
               {"BAR", "outcome3"},
           }},
          {"outcome2",
           {
               {"FOO", "outcome2"},
               {"BAR", "outcome3"},
           }},
      });
  concurrent_state->set_description(
      "Executes FooState and BarState in parallel and maps their combined "
      "outcomes to the next transition.");
  concurrent_state->add_input_key(
      "foo_str", "String read by BarState during the concurrent execution.");
  concurrent_state->add_output_key(
      "foo_str", "String written by FooState during the concurrent execution.");

  // Add concurrent state to the state machine
  sm->add_state("CONCURRENCE", concurrent_state,
                {
                    {"outcome1", "CONCURRENCE"},
                    {"outcome2", "outcome4"},
                    {"defaulted", "outcome4"},
                });

  {
    // Publish state machine updates
    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_CONCURRENCE_DEMO");

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
