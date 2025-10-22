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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

using namespace yasmin;

class FooState : public State {
public:
  FooState() : State({"outcome1"}) {}

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override {
    return "outcome1";
  }
};

class BarState : public State {
public:
  BarState() : State({}) {}

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override {
    return "outcome2";
  }
};

class TestState : public ::testing::Test {
protected:
  std::shared_ptr<FooState> state;
  std::shared_ptr<blackboard::Blackboard> blackboard;

  void SetUp() override {
    state = std::make_shared<FooState>();
    blackboard = std::make_shared<blackboard::Blackboard>();
  }
};

TEST_F(TestState, TestCall) { EXPECT_EQ((*state)(blackboard), "outcome1"); }

TEST_F(TestState, TestCancel) {
  EXPECT_FALSE(state->is_canceled());
  state->cancel_state();
  EXPECT_TRUE(state->is_canceled());
}

TEST_F(TestState, TestGetOutcomes) {
  auto outcomes = state->get_outcomes();
  EXPECT_EQ(outcomes.size(), 1);
  EXPECT_EQ(*outcomes.begin(), "outcome1");
}

TEST_F(TestState, TestStr) {
  std::string state_str = state->to_string();
  EXPECT_TRUE(state_str.find("FooState") != std::string::npos);
}

TEST_F(TestState, TestInitException) {
  try {
    BarState();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ("A state must have at least one possible outcome.", e.what());
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
