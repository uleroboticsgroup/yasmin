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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/foo_state.h"
#include "yasmin_ros/ros_logs.hpp"

FooState::FooState() : yasmin::State({"outcome1", "outcome2"}) { counter = 0; };

std::string FooState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state FOO");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (this->counter < 3) {
    this->counter += 1;
    blackboard->set<std::string>("foo_str",
                                 "Counter: " + std::to_string(this->counter));
    return "outcome1";

  } else {
    return "outcome2";
  }
};

FooState::~FooState() {};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(FooState, yasmin::State)