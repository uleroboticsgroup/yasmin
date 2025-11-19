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

#include <chrono>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state.hpp"

using namespace yasmin;

class FooState : public State {
public:
  FooState() : State({"outcome1"}) {}

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Foo state ticked." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Foo state ended." << std::endl;
    return "outcome1";
  }
};

class BarState : public State {
public:
  BarState() : State({"outcome1", "outcome2"}) {}

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Bar state ticked." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Bar state ended." << std::endl;
    return "outcome2";
  }
};

class TestConcurrence : public ::testing::Test {
protected:
  std::shared_ptr<FooState> foo_state;
  std::shared_ptr<FooState> foo2_state;
  std::shared_ptr<BarState> bar_state;
  std::shared_ptr<Concurrence> state;
  std::shared_ptr<blackboard::Blackboard> blackboard;

  void SetUp() override {
    foo_state = std::make_shared<FooState>();
    foo2_state = std::make_shared<FooState>();
    bar_state = std::make_shared<BarState>();
    blackboard = std::make_shared<blackboard::Blackboard>();

    std::map<std::string, std::shared_ptr<State>> states = {
        {"FOO", foo_state}, {"FOO2", foo2_state}, {"BAR", bar_state}};

    Concurrence::OutcomeMap outcome_map = {
        {"outcome1", {{"FOO", "outcome1"}}},
        {"outcome2", {{"BAR", "outcome1"}, {"BAR", "outcome1"}}}};

    state =
        std::make_shared<yasmin::Concurrence>(states, "default", outcome_map);
  }
};

TEST_F(TestConcurrence, TestCall) {
  EXPECT_EQ((*state)(blackboard), "outcome1");
}

TEST_F(TestConcurrence, TestCancel) {
  EXPECT_FALSE(state->is_canceled());
  state->cancel_state();
  EXPECT_TRUE(state->is_canceled());
}

TEST_F(TestConcurrence, TestStr) {
  std::string state_str = state->to_string();
  EXPECT_TRUE(state_str ==
              "Concurrence [BAR (BarState), FOO (FooState), FOO2 (FooState)]");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
