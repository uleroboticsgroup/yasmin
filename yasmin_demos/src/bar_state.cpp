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
#include "yasmin_demos/bar_state.h"
#include "yasmin_ros/ros_logs.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
BarState::BarState() : yasmin::State({"outcome3"}){};

/**
 * @brief Executes the Bar state logic.
 *
 * This method logs the execution, waits for 3 seconds,
 * retrieves a string from the blackboard, and logs it.
 *
 * @param blackboard Shared pointer to the blackboard for state communication.
 * @return std::string The outcome of the execution: "outcome3".
 */
std::string
BarState::execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  YASMIN_LOG_INFO("Executing state BAR");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

  return "outcome3";
};

BarState::~BarState(){};