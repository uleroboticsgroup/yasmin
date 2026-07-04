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

#include "yasmin_demos/bar_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"

BarState::BarState() : yasmin::State({"outcome3"}) {
  this->log_prefix_ = "Observed value";
  this->sleep_ms_ = 300;
  this->set_description("Prints the value stored in 'foo_str' from the "
                        "blackboard and transitions back to the Foo state.");
  this->set_outcome_description("outcome3", "Final outcome");
  this->declare_parameter<std::string>(
      "log_prefix", "Prefix printed before the blackboard value.",
      std::string("Observed value"));
  this->declare_parameter<int>(
      "sleep_ms", "Delay in milliseconds before each execution.", 300);
  this->add_input_key<std::string>(
      "foo_str", "String produced by FooState and printed by this state.",
      std::string("Foo"));
}

void BarState::configure() {
  this->log_prefix_ = this->get_parameter<std::string>("log_prefix");
  this->sleep_ms_ = this->get_parameter<int>("sleep_ms");
}

std::string BarState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state BAR");
  std::this_thread::sleep_for(std::chrono::milliseconds(this->sleep_ms_));

  YASMIN_LOG_INFO(
      (this->log_prefix_ + ": " + blackboard->get<std::string>("foo_str"))
          .c_str());

  return "outcome3";
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(BarState, yasmin::State)
