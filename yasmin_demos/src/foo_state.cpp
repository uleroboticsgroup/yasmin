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

#include "yasmin_demos/foo_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"

FooState::FooState() : yasmin::State({"outcome1", "outcome2"}) {
  this->counter = 0;
  this->counter_prefix_ = "Counter";
  this->max_count_ = 3;
  this->sleep_ms_ = 300;
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
  this->counter_prefix_ = this->get_parameter<std::string>("counter_prefix");
  this->max_count_ = this->get_parameter<int>("max_count");
  this->sleep_ms_ = this->get_parameter<int>("sleep_ms");
}

std::string FooState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state FOO");
  std::this_thread::sleep_for(std::chrono::milliseconds(this->sleep_ms_));

  if (this->counter < this->max_count_) {
    this->counter += 1;
    blackboard->set<std::string>("foo_str", this->counter_prefix_ + ": " +
                                                std::to_string(this->counter));
    return "outcome1";

  } else {
    return "outcome2";
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(FooState, yasmin::State)
