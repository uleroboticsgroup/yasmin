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

#include "yasmin/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

class FooState : public State {
public:
  FooState() : State({"outcome1"}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
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

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Bar state ticked." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Bar state ended." << std::endl;
    return "outcome2";
  }
};

class TestConcurrence : public ::testing::Test {
protected:
  yasmin::State::SharedPtr foo_state;
  yasmin::State::SharedPtr foo2_state;
  yasmin::State::SharedPtr bar_state;
  yasmin::Concurrence::SharedPtr state;
  yasmin::Blackboard::SharedPtr blackboard;

  void SetUp() override {
    foo_state = std::make_shared<FooState>();
    foo2_state = std::make_shared<FooState>();
    bar_state = std::make_shared<BarState>();
    blackboard = yasmin::Blackboard::make_shared();

    yasmin::StateMap states = {
        {"FOO", foo_state}, {"FOO2", foo2_state}, {"BAR", bar_state}};

    yasmin::OutcomeMap outcome_map = {
        {"outcome1", {{"FOO", "outcome1"}}},
        {"outcome2", {{"BAR", "outcome1"}, {"BAR", "outcome1"}}}};

    state = yasmin::Concurrence::make_shared(states, "default", outcome_map);
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

  // Check if "BAR (BarState)" is in the string
  EXPECT_TRUE(state_str.find("BAR (BarState)") != std::string::npos);
  EXPECT_TRUE(state_str.find("FOO (FooState)") != std::string::npos);
  EXPECT_TRUE(state_str.find("FOO2 (FooState)") != std::string::npos);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
