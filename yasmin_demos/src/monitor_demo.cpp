// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PrintOdometryState
    : public yasmin_ros::MonitorState<nav_msgs::msg::Odometry> {

public:
  int times;

  PrintOdometryState(int times)
      : yasmin_ros::MonitorState<nav_msgs::msg::Odometry> // msg type
        ("odom",                                          // topic name
         {"outcome1", "outcome2"},                        // outcomes
         std::bind(&PrintOdometryState::monitor_handler, this, _1,
                   _2), // monitor handler callback
         10,            // qos for the topic sbscription
         10,            // queue of the monitor handler callback
         10             // timeout to wait for msgs in seconds
                        // if >0, CANCEL outcome is added
        ) {
    this->times = times;
  };

  std::string
  monitor_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                  std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    (void)blackboard;

    std::cout << "x: " << msg->pose.pose.position.x << "\n";
    std::cout << "y: " << msg->pose.pose.position.y << "\n";
    std::cout << "z: " << msg->pose.pose.position.z << "\n";
    std::cout << "\n";

    this->times--;

    if (this->times <= 0) {
      return "outcome2";
    }

    return "outcome1";
  }

  std::string to_string() { return "PrintOdometryState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_monitor_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // add states
  sm->add_state("PRINTING_ODOM", std::make_shared<PrintOdometryState>(5),
                {{"outcome1", "PRINTING_ODOM"},
                 {"outcome2", "outcome4"},
                 {yasmin_ros::basic_outcomes::TIMEOUT, "outcome4"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_MONITOR_DEMO", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
