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

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
FooState::FooState() : yasmin::State({"outcome1", "outcome2"}) { counter = 0; };

/**
 * @brief Executes the Foo state logic.
 *
 * This method logs the execution, waits for 3 seconds,
 * increments the counter, and sets a string in the blackboard.
 * The state will transition to either "outcome1" or "outcome2"
 * based on the current value of the counter.
 *
 * @param blackboard Shared pointer to the blackboard for state communication.
 * @return std::string The outcome of the execution: "outcome1" or "outcome2".
 */
std::string
FooState::execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
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

FooState::~FooState(){};
