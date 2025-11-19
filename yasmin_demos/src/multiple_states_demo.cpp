// Copyright (C) 2025 Pedro Edom Nunes
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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/bar_state.h"
#include "yasmin_demos/foo_state.h"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_multiple_states_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

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

  // Publisher for visualizing the state machine
  yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_MULTIPLE_STATES_DEMO");

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
