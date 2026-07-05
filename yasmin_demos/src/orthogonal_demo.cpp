// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

class WorkerState : public yasmin::State {
public:
  int counter;
  int max_count_;
  int sleep_ms_;
  std::string name_;

  WorkerState(const std::string &name, int max_count, int sleep_ms = 500)
      : yasmin::State({"working", "done"}), counter(0), max_count_(max_count),
        sleep_ms_(sleep_ms), name_(name) {
    this->set_description(
        "Counts iterations and returns 'working' until max_count is reached.");
  };

  std::string execute(yasmin::Blackboard::SharedPtr /*blackboard*/) override {
    YASMIN_LOG_INFO("WorkerState [%s]: iteration %d/%d", this->name_.c_str(),
                    this->counter + 1, this->max_count_);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->sleep_ms_));

    this->counter += 1;
    if (this->counter >= this->max_count_) {
      return "done";
    }
    return "working";
  };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  yasmin_ros::set_ros_loggers();
  YASMIN_LOG_INFO("yasmin_orthogonal_demo");

  // Region A: 3 iterations
  auto reg_a = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{"done"});
  reg_a->set_name("Region A");
  reg_a->add_state("WORK", std::make_shared<WorkerState>("A", 3, 300),
                   {{"working", "WORK"}});
  reg_a->set_start_state("WORK");

  // Region B: 5 iterations
  auto reg_b = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{"done"});
  reg_b->set_name("Region B");
  reg_b->add_state("WORK", std::make_shared<WorkerState>("B", 5, 300),
                   {{"working", "WORK"}});
  reg_b->set_start_state("WORK");

  // Orthogonal state runs both regions in parallel
  auto ort = yasmin::OrthogonalState::make_shared(
      "timeout", yasmin::OutcomeMap{
                     {"success", {{"A", "done"}, {"B", "done"}}},
                 });
  ort->set_description("Runs Region A and Region B in parallel and maps their "
                       "combined outcomes.");
  ort->add_region("A", reg_a);
  ort->add_region("B", reg_b);

  // Root state machine
  auto sm = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{"success", "timeout"}, true);
  sm->set_description(
      "Demonstrates orthogonal state execution with two parallel regions.");
  sm->add_state("PARALLEL", ort,
                {{"success", "success"}, {"timeout", "timeout"}});
  sm->set_start_state("PARALLEL");

  {
    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_ORTHOGONAL_DEMO");

    try {
      std::string outcome = (*sm.get())();
      YASMIN_LOG_INFO(outcome.c_str());
    } catch (const std::exception &e) {
      YASMIN_LOG_WARN(e.what());
    }
  }

  yasmin_ros::YasminNode::destroy_instance();
  rclcpp::shutdown();
  return 0;
}
