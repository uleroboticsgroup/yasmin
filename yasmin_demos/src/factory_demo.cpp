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

#include "yasmin/state_machine.hpp"
#include "yasmin_demos/share_directory.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();
  YASMIN_LOG_INFO("yasmin_factory_demo");

  // Create the factory and state machine in a scope to ensure proper cleanup
  yasmin_factory::YasminFactory factory;

  // Load state machine from XML file
  std::string xml_file =
      yasmin_demos::get_share_file_path("state_machines/demo_2.xml");

  // Create the state machine from the XML file
  auto sm = factory.create_sm_from_file(xml_file);
  sm->set_sigint_handler(true);

  {
    // Publisher for visualizing the state machine
    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_FACTORY_DEMO");

    // Execute the state machine
    try {
      std::string outcome = (*sm.get())();
      YASMIN_LOG_INFO(outcome.c_str());
    } catch (const std::exception &e) {
      YASMIN_LOG_WARN(e.what());
    }
  }

  yasmin_ros::YasminNode::destroy_instance();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
