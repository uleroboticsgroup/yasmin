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

#include <pluginlib/class_list_macros.hpp>

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

PLUGINLIB_EXPORT_CLASS(TestSimpleState, yasmin::State)
