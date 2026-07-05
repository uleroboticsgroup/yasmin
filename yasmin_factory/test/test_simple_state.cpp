// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <memory>
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

/**
 * @brief Simple test state for testing purposes.
 */
class TestSimpleState : public yasmin::State {
public:
  TestSimpleState() : yasmin::State({"outcome1", "outcome2"}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {

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

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {

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
