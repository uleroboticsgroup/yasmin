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
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_factory_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  std::string outcome;

  // Create the factory and state machine in a scope to ensure proper cleanup
  yasmin_factory::YasminFactory factory;

  // Load state machine from XML file
  std::string xml_file =
      ament_index_cpp::get_package_share_directory("yasmin_demos") +
      "/state_machines/demo_2.xml";

  // Create the state machine from the XML file
  auto sm = factory.create_sm_from_file(xml_file);

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
  yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_FACTORY_DEMO");

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
