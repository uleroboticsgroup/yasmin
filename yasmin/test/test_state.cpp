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
    add_input_key<int>("counter", "A counter", 42);
    add_input_key<std::string>("label", "A Label", std::string("hello"));
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
  BlackboardKeyInfo info("speed", "Maximum robot speed in m/s", 1.5);

  auto s = std::make_shared<FooState>();
  s->add_input_key(info);

  const auto &keys = s->get_input_keys();
  ASSERT_EQ(keys.size(), 1u);
  EXPECT_EQ(keys[0].name, "speed");
  EXPECT_EQ(keys[0].description, "Maximum robot speed in m/s");
  EXPECT_TRUE(keys[0].has_default);
}

// ---------------------------------------------------------------------------
// Additional metadata tests for various types and get_default_value
// ---------------------------------------------------------------------------

class StateWithAllTypes : public State {
public:
  StateWithAllTypes() : State({"done"}) {
    set_description("State with various default types");
    set_outcome_description("done", "Main outcome");
    add_input_key<bool>("flag", "A flag", true);
    add_input_key<double>("speed", "Speed", 3.14);
    add_input_key<int>("count", "A counter", 10);
    add_input_key<std::string>("name", "Name", std::string("robot"));
    add_output_key("result");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "done";
  }
};

TEST_F(TestState, TestDescriptionSetAndGet) {
  StateWithAllTypes s;
  EXPECT_EQ(s.get_description(), "State with various default types");
}

TEST_F(TestState, TestOutcomeDescription) {
  StateWithAllTypes s;
  EXPECT_EQ(s.get_outcome_description("done"), "Main outcome");
}

TEST_F(TestState, TestMultipleInputKeyTypes) {
  StateWithAllTypes s;
  const auto &keys = s.get_input_keys();
  ASSERT_EQ(keys.size(), 4u);

  // bool
  EXPECT_EQ(keys[0].name, "flag");
  EXPECT_TRUE(keys[0].has_default);
  EXPECT_EQ(keys[0].get_default_value<bool>(), true);

  // double
  EXPECT_EQ(keys[1].name, "speed");
  EXPECT_TRUE(keys[1].has_default);
  EXPECT_NEAR(keys[1].get_default_value<double>(), 3.14, 0.001);

  // int
  EXPECT_EQ(keys[2].name, "count");
  EXPECT_TRUE(keys[2].has_default);
  EXPECT_EQ(keys[2].get_default_value<int>(), 10);

  // string
  EXPECT_EQ(keys[3].name, "name");
  EXPECT_TRUE(keys[3].has_default);
  EXPECT_EQ(keys[3].get_default_value<std::string>(), "robot");
}

TEST_F(TestState, TestOutputKey) {
  StateWithAllTypes s;
  const auto &keys = s.get_output_keys();
  ASSERT_EQ(keys.size(), 1u);

  EXPECT_EQ(keys[0].name, "result");
  EXPECT_FALSE(keys[0].has_default);
}

TEST_F(TestState, TestAllDefaultsInjected) {
  StateWithAllTypes s;
  auto bb = yasmin::Blackboard::make_shared();
  s(bb);

  EXPECT_EQ(bb->get<bool>("flag"), true);
  EXPECT_NEAR(bb->get<double>("speed"), 3.14, 0.001);
  EXPECT_EQ(bb->get<int>("count"), 10);
  EXPECT_EQ(bb->get<std::string>("name"), "robot");
}

TEST_F(TestState, TestGetMetadataComplete) {
  StateWithAllTypes s;
  const auto &meta = s.get_metadata();
  EXPECT_EQ(meta.description, "State with various default types");
  EXPECT_EQ(meta.input_keys.size(), 4u);
  EXPECT_EQ(meta.output_keys.size(), 1u);
}

TEST_F(TestState, TestBlackboardKeyInfoCopy) {
  BlackboardKeyInfo info("val", "A value", 42);
  BlackboardKeyInfo copy = info;
  EXPECT_EQ(copy.name, "val");
  EXPECT_TRUE(copy.has_default);
  EXPECT_EQ(copy.get_default_value<int>(), 42);

  // Injection through copy should work
  auto bb = yasmin::Blackboard::make_shared();
  copy.inject_default(*bb, "val");
  EXPECT_EQ(bb->get<int>("val"), 42);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
