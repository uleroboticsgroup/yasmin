// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <memory>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

/**
 * @brief Simple test state for testing purposes.
 */
class TestSimpleState : public yasmin::State {
public:
  TestSimpleState() : yasmin::State({"outcome1", "outcome2"}) {}

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    if (!blackboard->contains("counter")) {
      blackboard->set<int>("counter", 0);
    }

    int value = blackboard->get<int>("counter");
    blackboard->set<int>("counter", value + 1);
    return value < 2 ? "outcome1" : "outcome2";
  }
};

/**
 * @brief Test state that reads from and writes to specific blackboard keys.
 */
class TestRemappingState : public yasmin::State {
public:
  TestRemappingState() : yasmin::State({"success", "failure"}) {}

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    if (blackboard->contains("input_key")) {
      std::string value = blackboard->get<std::string>("input_key");
      // Write to output_key (can also be remapped independently)
      blackboard->set<std::string>("output_key", "processed_" + value);
      return "success";
    }
    return "failure";
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TestSimpleState, yasmin::State)
PLUGINLIB_EXPORT_CLASS(TestRemappingState, yasmin::State)
