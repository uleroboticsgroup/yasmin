// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include "yasmin_demos/worker_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"

WorkerState::WorkerState() : yasmin::State({"working", "done"}) {
  this->counter = 0;
  this->max_count_ = 3;
  this->sleep_ms_ = 500;
  this->set_description(
      "Counts iterations and returns 'working' until max_count is reached.");
  this->set_outcome_description("working", "Counter is below the threshold");
  this->set_outcome_description("done", "Counter reached the threshold.");
  this->declare_parameter<int>(
      "max_count", "Number of iterations before the state returns 'done'.", 3);
  this->declare_parameter<int>(
      "sleep_ms", "Delay in milliseconds before each execution.", 500);
};

void WorkerState::configure() {
  this->max_count_ = this->get_parameter<int>("max_count");
  this->sleep_ms_ = this->get_parameter<int>("sleep_ms");
  this->counter = 0;
}

std::string WorkerState::execute(yasmin::Blackboard::SharedPtr /*blackboard*/) {
  YASMIN_LOG_INFO("Executing WorkerState: iteration %d/%d", this->counter + 1,
                  this->max_count_);
  std::this_thread::sleep_for(std::chrono::milliseconds(this->sleep_ms_));

  this->counter += 1;
  if (this->counter >= this->max_count_) {
    return "done";
  }
  return "working";
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(WorkerState, yasmin::State)
