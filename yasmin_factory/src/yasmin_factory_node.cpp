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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_factory_node");
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  // Get the state machine file parameter
  auto node = yasmin_ros::YasminNode::get_instance();
  node->declare_parameter("state_machine_file", "");
  std::string sm_file = node->get_parameter("state_machine_file")
                            .get_parameter_value()
                            .get<std::string>();

  // Create the factory in a scope
  yasmin_factory::YasminFactory factory;

  // Create the state machine from the XML file
  auto sm = factory.create_sm_from_file(sm_file);

  // Cancel state machine on ROS 2 shutdown
  std::weak_ptr<yasmin::StateMachine> weak_sm = sm;
  rclcpp::on_shutdown([weak_sm]() {
    if (auto sm = weak_sm.lock()) {
      if (sm->is_running()) {
        sm->cancel_state();
      }
    }
  });

  // Publisher for visualizing the state machine
  yasmin_viewer::YasminViewerPub yasmin_pub(sm);

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
