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

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

using namespace yasmin;

class FooState : public State {
public:
  FooState() : State({"outcome1"}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "outcome1";
  }
};

class BarState : public State {
public:
  BarState() : State({}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "outcome2";
  }
};

class TestState : public ::testing::Test {
protected:
  FooState::SharedPtr state;
  yasmin::Blackboard::SharedPtr blackboard;

  void SetUp() override {
    state = std::make_shared<FooState>();
    blackboard = yasmin::Blackboard::make_shared();
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
  EXPECT_TRUE(state_str == "FooState");
}

TEST_F(TestState, TestInitException) {
  try {
    BarState();
    FAIL() << "Expected std::logic_error";
  } catch (const std::logic_error &e) {
    EXPECT_STREQ("A state must have at least one possible outcome.", e.what());
  }
}

// ---------------------------------------------------------------------------
// Metadata tests
// ---------------------------------------------------------------------------
class StateWithDefaults : public State {
public:
  StateWithDefaults() : State({"done"}) {
    add_input_key<int>("counter", 42);
    add_input_key<std::string>("label", std::string("hello"));
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "done";
  }
};

TEST_F(TestState, TestDefaultValueInjectedWhenMissing) {
  StateWithDefaults s;
  auto bb = yasmin::Blackboard::make_shared();
  s(bb);
  EXPECT_EQ(bb->get<int>("counter"), 42);
  EXPECT_EQ(bb->get<std::string>("label"), "hello");
}

TEST_F(TestState, TestDefaultValueNotOverwrittenWhenPresent) {
  StateWithDefaults s;
  auto bb = yasmin::Blackboard::make_shared();
  bb->set<int>("counter", 100);
  s(bb);
  EXPECT_EQ(bb->get<int>("counter"), 100);
}

TEST_F(TestState, TestDefaultValueRespectsMappedKeyExists) {
  StateWithDefaults s;
  auto bb = yasmin::Blackboard::make_shared();
  // "counter" is remapped to "my_counter"; the remapped key already exists
  bb->set_remappings({{"counter", "my_counter"}});
  bb->set<int>("my_counter", 99);
  s(bb);
  EXPECT_EQ(bb->get<int>("my_counter"), 99);
}

TEST_F(TestState, TestDefaultValueInjectedOnRemappedKey) {
  StateWithDefaults s;
  auto bb = yasmin::Blackboard::make_shared();
  // "counter" remapped to "my_counter"; nothing in the blackboard yet
  bb->set_remappings({{"counter", "my_counter"}});
  s(bb);
  EXPECT_EQ(bb->get<int>("my_counter"), 42);
}

TEST_F(TestState, TestDefaultValueEachCallIsIndependent) {
  StateWithDefaults s;
  auto bb1 = yasmin::Blackboard::make_shared();
  auto bb2 = yasmin::Blackboard::make_shared();
  s(bb1);
  bb1->set<int>("counter", 7);
  s(bb2);
  // Modifying bb1 must not affect the injected default in bb2
  EXPECT_EQ(bb2->get<int>("counter"), 42);
}

TEST_F(TestState, TestKeyInfoDescription) {
  BlackboardKeyInfo info("speed", 1.5);
  info.description = "Maximum robot speed in m/s";

  auto s = std::make_shared<FooState>();
  s->add_input_key(info);

  const auto &keys = s->get_input_keys();
  ASSERT_EQ(keys.size(), 1u);
  EXPECT_EQ(keys[0].name, "speed");
  EXPECT_EQ(keys[0].description, "Maximum robot speed in m/s");
  EXPECT_TRUE(keys[0].has_default);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
