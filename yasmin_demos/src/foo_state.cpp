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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/foo_state.h"
#include "yasmin_ros/ros_logs.hpp"

FooState::FooState() : yasmin::State({"outcome1", "outcome2"}) {
  counter = 0;
  counter_prefix_ = "Counter";
  max_count_ = 3;
  sleep_ms_ = 300;
  this->set_description("Produces a counter string and stores it in the "
                        "blackboard while the counter is below the threshold.");
  this->set_outcome_description("outcome1", "Counter is below the threshold");
  this->set_outcome_description("outcome2", "Counter reached the threshold.");
  this->declare_parameter<std::string>(
      "counter_prefix", "Prefix used when formatting the counter string.",
      std::string("Counter"));
  this->declare_parameter<int>(
      "max_count",
      "Number of successful loops before the state returns outcome2.", 3);
  this->declare_parameter<int>(
      "sleep_ms", "Delay in milliseconds before each execution.", 300);
  this->add_output_key(
      "foo_str",
      "String containing the current counter value produced by FooState.");
};

void FooState::configure() {
  counter_prefix_ = this->get_parameter<std::string>("counter_prefix");
  max_count_ = this->get_parameter<int>("max_count");
  sleep_ms_ = this->get_parameter<int>("sleep_ms");
}

std::string FooState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state FOO");
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));

  if (this->counter < max_count_) {
    this->counter += 1;
    blackboard->set<std::string>("foo_str", counter_prefix_ + ": " +
                                                std::to_string(this->counter));
    return "outcome1";

  } else {
    return "outcome2";
  }
};

FooState::~FooState() {};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(FooState, yasmin::State)
