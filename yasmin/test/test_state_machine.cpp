// Copyright (C) 2023 Miguel Ángel González Santamarta
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
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

class FooState : public State {
private:
  int counter;

public:
  FooState() : State({"outcome1", "outcome2"}), counter(0) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    if (counter < 3) {
      counter++;
      blackboard->set<std::string>("foo_str",
                                   "Counter: " + std::to_string(counter));
      return "outcome1";
    } else {
      return "outcome2";
    }
  }
};

class BarState : public State {
public:
  BarState() : State({"outcome2", "outcome3"}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "outcome2";
  }
};

class TestStateMachine : public ::testing::Test {
protected:
  yasmin::StateMachine::SharedPtr sm;
  yasmin::Blackboard::SharedPtr blackboard;

  void SetUp() override {
    sm = StateMachine::make_shared(yasmin::Outcomes{"outcome4", "outcome5"});
    blackboard = yasmin::Blackboard::make_shared();

    sm->add_state(
        "FOO", std::make_shared<FooState>(),
        yasmin::Transitions{{"outcome1", "BAR"}, {"outcome2", "outcome4"}});

    sm->add_state("BAR", std::make_shared<BarState>(),
                  yasmin::Transitions{{"outcome2", "FOO"}});
  }
};

TEST_F(TestStateMachine, TestStr) {
  std::string sm_str = sm->to_string();
  // Check if "Bar (BarState)" and "Foo (FooState)" are in the string
  EXPECT_TRUE(sm_str.find("BAR (BarState)") != std::string::npos);
  EXPECT_TRUE(sm_str.find("FOO (FooState)") != std::string::npos);
}

TEST_F(TestStateMachine, TestGetNameEmpty) { EXPECT_EQ(sm->get_name(), ""); }

TEST_F(TestStateMachine, TestSetName) {
  sm->set_name("TestSM");
  EXPECT_EQ(sm->get_name(), "TestSM");
}

TEST_F(TestStateMachine, TestGetStates) {
  auto states = sm->get_states();
  EXPECT_EQ(states.size(), 2);
  EXPECT_TRUE(states.find("FOO") != states.end());
  EXPECT_TRUE(states.find("BAR") != states.end());
}

TEST_F(TestStateMachine, TestGetStartState) {
  EXPECT_EQ(sm->get_start_state(), "FOO");
  sm->set_start_state("BAR");
  EXPECT_EQ(sm->get_start_state(), "BAR");
}

TEST_F(TestStateMachine, TestGetCurrentState) {
  EXPECT_EQ(sm->get_current_state(), "");
}

TEST_F(TestStateMachine, TestStateCall) {
  EXPECT_EQ((*sm)(blackboard), "outcome4");
}

TEST_F(TestStateMachine, TestSetStartStateEmpty) {
  try {
    sm->set_start_state("");
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ("Initial state cannot be empty", e.what());
  }
}

TEST_F(TestStateMachine, TestSetStartStateWrongState) {
  try {
    sm->set_start_state("FOO1");
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ("Initial state 'FOO1' is not in the state machine", e.what());
  }
}

TEST_F(TestStateMachine, TestAddRepeatedState) {
  try {
    sm->add_state("FOO", std::make_shared<FooState>(),
                  yasmin::Transitions{{"outcome1", "BAR"}});
    FAIL() << "Expected std::logic_error";
  } catch (const std::logic_error &e) {
    EXPECT_STREQ("State 'FOO' already registered in the state machine",
                 e.what());
  }
}

TEST_F(TestStateMachine, TestAddOutcomeState) {
  try {
    sm->add_state("outcome4", std::make_shared<FooState>(),
                  yasmin::Transitions{{"outcome1", "BAR"}});
    FAIL() << "Expected std::logic_error";
  } catch (const std::logic_error &e) {
    EXPECT_STREQ("State name 'outcome4' is already registered as an outcome",
                 e.what());
  }
}

TEST_F(TestStateMachine, TestAddStateWithWrongOutcome) {
  try {
    sm->add_state("FOO1", std::make_shared<FooState>(),
                  yasmin::Transitions{{"outcome9", "BAR"}});
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    std::string error_msg = e.what();
    EXPECT_TRUE(error_msg.find("State 'FOO1' references unregistered outcomes "
                               "'outcome9'") != std::string::npos);
    EXPECT_TRUE(error_msg.find("available outcomes are") != std::string::npos);
  }
}

TEST_F(TestStateMachine, TestAddWrongSourceTransition) {
  try {
    sm->add_state("FOO1", std::make_shared<FooState>(),
                  yasmin::Transitions{{"", "BAR"}});
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ("Transitions with empty source in state 'FOO1'", e.what());
  }
}

TEST_F(TestStateMachine, TestAddWrongTargetTransition) {
  try {
    sm->add_state("FOO1", std::make_shared<FooState>(),
                  yasmin::Transitions{{"outcome1", ""}});
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ("Transitions with empty target in state 'FOO1'", e.what());
  }
}

TEST_F(TestStateMachine, TestValidateOutcomeFromFsmNotUsed) {
  auto sm1 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  auto sm2 =
      StateMachine::make_shared(yasmin::Outcomes{"outcome4", "outcome5"});
  sm1->add_state("FSM", sm2);
  sm2->add_state("FOO", std::make_shared<FooState>(),
                 {
                     {"outcome1", "outcome4"},
                     {"outcome2", "outcome4"},
                 });

  try {
    sm1->validate(true);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_STREQ("State 'FSM' outcome 'outcome5' not registered in transitions",
                 e.what());
  }
}

TEST_F(TestStateMachine, TestValidateOutcomeFromStateNotUsed) {
  auto sm1 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  auto sm2 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  sm1->add_state("FSM", sm2);
  sm2->add_state("FOO", std::make_shared<FooState>(),
                 {
                     {"outcome1", "outcome4"},
                 });

  try {
    sm1->validate(true);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_STREQ("State 'FOO' outcome 'outcome2' not registered in transitions",
                 e.what());
  }
}

TEST_F(TestStateMachine, TestValidateFsmOutcomeNotUsed) {
  auto sm1 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  auto sm2 =
      StateMachine::make_shared(yasmin::Outcomes{"outcome4", "outcome5"});
  sm1->add_state("FSM", sm2,
                 {
                     {"outcome5", "outcome4"},
                 });
  sm2->add_state("FOO", std::make_shared<FooState>(),
                 {
                     {"outcome1", "outcome4"},
                     {"outcome2", "outcome4"},
                 });

  try {
    sm1->validate(true);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_STREQ("Target outcome 'outcome5' not registered in transitions",
                 e.what());
  }
}

TEST_F(TestStateMachine, TestValidateWrongState) {
  auto sm1 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  auto sm2 = StateMachine::make_shared(yasmin::Outcomes{"outcome4"});
  sm1->add_state("FSM", sm2,
                 {
                     {"outcome4", "outcome4"},
                 });
  sm2->add_state("FOO", std::make_shared<FooState>(),
                 {
                     {"outcome1", "BAR"},
                     {"outcome2", "outcome4"},
                 });

  try {
    sm1->validate();
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_STREQ(
        "State machine outcome 'BAR' not registered as outcome neither state",
        e.what());
  }
}

TEST_F(TestStateMachine, TestStartCallback) {
  bool start_called = false;
  std::string start_state_called;

  sm->add_start_cb([&](yasmin::Blackboard::SharedPtr blackboard,
                       const std::string &start_state) {
    start_called = true;
    start_state_called = start_state;
  });

  EXPECT_EQ((*sm)(blackboard), "outcome4");
  EXPECT_TRUE(start_called);
  EXPECT_EQ(start_state_called, "FOO");
}

TEST_F(TestStateMachine, TestTransitionCallback) {
  std::vector<std::tuple<std::string, std::string, std::string>>
      transitions_called;

  sm->add_transition_cb([&](yasmin::Blackboard::SharedPtr blackboard,
                            const std::string &from_state,
                            const std::string &to_state,
                            const std::string &outcome) {
    transitions_called.emplace_back(from_state, to_state, outcome);
  });

  EXPECT_EQ((*sm)(blackboard), "outcome4");

  // Should have transitions: FOO -> BAR (outcome1), BAR -> FOO (outcome2), FOO
  // -> BAR (outcome1), BAR -> FOO (outcome2), FOO -> BAR (outcome1), BAR -> FOO
  // (outcome2)
  EXPECT_EQ(transitions_called.size(), 6);
  EXPECT_EQ(std::get<0>(transitions_called[0]), "FOO");
  EXPECT_EQ(std::get<1>(transitions_called[0]), "BAR");
  EXPECT_EQ(std::get<2>(transitions_called[0]), "outcome1");
  EXPECT_EQ(std::get<0>(transitions_called[1]), "BAR");
  EXPECT_EQ(std::get<1>(transitions_called[1]), "FOO");
  EXPECT_EQ(std::get<2>(transitions_called[1]), "outcome2");
  EXPECT_EQ(std::get<0>(transitions_called[2]), "FOO");
  EXPECT_EQ(std::get<1>(transitions_called[2]), "BAR");
  EXPECT_EQ(std::get<2>(transitions_called[2]), "outcome1");
  EXPECT_EQ(std::get<0>(transitions_called[3]), "BAR");
  EXPECT_EQ(std::get<1>(transitions_called[3]), "FOO");
  EXPECT_EQ(std::get<2>(transitions_called[3]), "outcome2");
  EXPECT_EQ(std::get<0>(transitions_called[4]), "FOO");
  EXPECT_EQ(std::get<1>(transitions_called[4]), "BAR");
  EXPECT_EQ(std::get<2>(transitions_called[4]), "outcome1");
  EXPECT_EQ(std::get<0>(transitions_called[5]), "BAR");
  EXPECT_EQ(std::get<1>(transitions_called[5]), "FOO");
  EXPECT_EQ(std::get<2>(transitions_called[5]), "outcome2");
}

TEST_F(TestStateMachine, TestEndCallback) {
  bool end_called = false;
  std::string end_outcome_called;

  sm->add_end_cb([&](yasmin::Blackboard::SharedPtr blackboard,
                     const std::string &outcome) {
    end_called = true;
    end_outcome_called = outcome;
  });

  EXPECT_EQ((*sm)(blackboard), "outcome4");
  EXPECT_TRUE(end_called);
  EXPECT_EQ(end_outcome_called, "outcome4");
}

TEST_F(TestStateMachine, TestMultipleCallbacks) {
  int start_count = 0;
  int transition_count = 0;
  int end_count = 0;

  sm->add_start_cb([&](yasmin::Blackboard::SharedPtr, const std::string &) {
    start_count++;
  });
  sm->add_start_cb([&](yasmin::Blackboard::SharedPtr, const std::string &) {
    start_count++;
  });

  sm->add_transition_cb([&](yasmin::Blackboard::SharedPtr, const std::string &,
                            const std::string &,
                            const std::string &) { transition_count++; });
  sm->add_transition_cb([&](yasmin::Blackboard::SharedPtr, const std::string &,
                            const std::string &,
                            const std::string &) { transition_count++; });

  sm->add_end_cb(
      [&](yasmin::Blackboard::SharedPtr, const std::string &) { end_count++; });
  sm->add_end_cb(
      [&](yasmin::Blackboard::SharedPtr, const std::string &) { end_count++; });

  EXPECT_EQ((*sm)(blackboard), "outcome4");
  EXPECT_EQ(start_count, 2);
  EXPECT_EQ(transition_count, 12); // 6 transitions * 2 callbacks
  EXPECT_EQ(end_count, 2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
