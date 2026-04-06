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

#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "yasmin/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
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

class ConfigurableConcurrentState : public State {
public:
  int configure_count{0};
  std::string configured_topic;

  ConfigurableConcurrentState() : State({"done"}) {
    this->declare_parameter("topic", "Concurrent topic");
  }

  void configure() override {
    configure_count++;
    configured_topic = this->get_parameter<std::string>("topic");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    return "done";
  }
};

TEST_F(TestConcurrence, TestConfigureAppliesParameterMappingsAndRunsOnce) {
  auto child = std::make_shared<ConfigurableConcurrentState>();
  auto concurrent = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"CHILD", child}}, "done",
      yasmin::OutcomeMap{{"done", {{"CHILD", "done"}}}});
  concurrent->declare_parameter<std::string>("topic", "Parent topic",
                                             std::string("/concurrent"));
  concurrent->set_parameter_mappings("CHILD", {{"topic", "topic"}});

  EXPECT_EQ((*concurrent)(blackboard), "done");
  EXPECT_EQ((*concurrent)(blackboard), "done");
  EXPECT_EQ(child->configure_count, 1);
  EXPECT_EQ(child->configured_topic, "/concurrent");
}

struct ConcurrentWriteSync {
  std::mutex mutex;
  std::condition_variable condition;
  int ready_count{0};
  bool release_writes{false};
};

/**
 * @brief State that waits until both concurrent branches are ready and then
 * writes a value through the remapped blackboard key.
 *
 * The barrier is used to make both branches perform their write while both
 * nested state machines are active. This makes remapping interference visible
 * if the remapping context is shared between concurrent branches.
 */
class BlockingWriteState : public State {
public:
  BlockingWriteState(std::shared_ptr<ConcurrentWriteSync> sync,
                     const std::string &value)
      : State({"done"}), sync(std::move(sync)), value(value) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    {
      // Wait here until both branches have entered execute().
      // This ensures both writes happen while the concurrence container is
      // running both nested state machines at the same time.
      std::unique_lock<std::mutex> lock(this->sync->mutex);
      this->sync->ready_count++;
      this->sync->condition.notify_all();
      this->sync->condition.wait(
          lock, [this]() { return this->sync->release_writes; });
    }

    // Write through the remapped key. The final storage key depends on the
    // remapping chain of the current branch.
    blackboard->set<std::string>("foo_str", this->value);
    return "done";
  }

private:
  std::shared_ptr<ConcurrentWriteSync> sync;
  std::string value;
};

/**
 * @brief Small helper state machine that delays its start.
 *
 * The delay makes the scheduling of the two concurrent branches less symmetric,
 * which helps to reproduce bugs caused by shared remapping state.
 */
class DelayedStartStateMachine : public StateMachine {
public:
  explicit DelayedStartStateMachine(std::chrono::milliseconds start_delay)
      : StateMachine({"done"}), start_delay(start_delay) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    std::this_thread::sleep_for(this->start_delay);
    return StateMachine::execute(blackboard);
  }

private:
  std::chrono::milliseconds start_delay;
};

/**
 * @brief Build one concurrent branch consisting of two nested remapping levels.
 *
 * Layout:
 *   outer state machine: inner_key -> final_key
 *   inner state machine: foo_str   -> inner_key
 *
 * The leaf state always writes to "foo_str". The nested remappings should make
 * that value appear only under final_key in the shared blackboard.
 */
static yasmin::StateMachine::SharedPtr
make_nested_branch(const std::shared_ptr<ConcurrentWriteSync> &sync,
                   const std::string &inner_key, const std::string &final_key,
                   const std::string &value,
                   std::chrono::milliseconds start_delay) {
  auto outer_state_machine =
      std::make_shared<DelayedStartStateMachine>(start_delay);
  auto inner_state_machine =
      std::make_shared<yasmin::StateMachine>(yasmin::Outcomes{"done"});

  inner_state_machine->add_state(
      "WRITE", std::make_shared<BlockingWriteState>(sync, value),
      yasmin::Transitions{{"done", "done"}},
      yasmin::Remappings{{"foo_str", inner_key}});

  outer_state_machine->add_state("INNER", inner_state_machine,
                                 yasmin::Transitions{{"done", "done"}},
                                 yasmin::Remappings{{inner_key, final_key}});

  return outer_state_machine;
}

/**
 * @brief Verify that concurrent nested state machines keep remappings isolated.
 *
 * Each branch has its own nested remapping chain:
 *   Branch A: foo_str -> foo_inner_a -> foo_branch_a
 *   Branch B: foo_str -> foo_inner_b -> foo_branch_b
 *
 * Both branches run at the same time and write through "foo_str". If remapping
 * state is still shared globally, one branch can overwrite the remapping
 * context of the other and the values may end up under the wrong keys.
 *
 * The expected behavior is:
 * - each final branch key gets its own value
 * - no intermediate remapping keys remain visible
 * - the original key "foo_str" is not stored directly
 */
TEST_F(TestConcurrence,
       TestConcurrentNestedStateMachinesKeepRemappingsIsolatedPerBranch) {
  auto sync = std::make_shared<ConcurrentWriteSync>();

  // Build two concurrent branches with different nested remapping chains.
  auto branch_a = make_nested_branch(sync, "foo_inner_a", "foo_branch_a",
                                     "value_a", std::chrono::milliseconds(0));
  auto branch_b = make_nested_branch(sync, "foo_inner_b", "foo_branch_b",
                                     "value_b", std::chrono::milliseconds(50));

  auto concurrent = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"BRANCH_A", branch_a}, {"BRANCH_B", branch_b}},
      "failed",
      yasmin::OutcomeMap{
          {"done", {{"BRANCH_A", "done"}, {"BRANCH_B", "done"}}}});

  // Execute the concurrence container asynchronously so the test thread can
  // control when both branches are released to perform their writes.
  auto future = std::async(std::launch::async, [concurrent, this]() {
    return (*concurrent)(blackboard);
  });

  {
    std::unique_lock<std::mutex> lock(sync->mutex);

    // Wait until both leaf states are inside execute(). This guarantees that
    // the two writes happen while both branches are active.
    sync->condition.wait(lock, [sync]() { return sync->ready_count == 2; });

    // Release both branches together.
    sync->release_writes = true;
  }
  sync->condition.notify_all();

  EXPECT_EQ(future.get(), "done");

  // Each branch must write to its own final remapped key.
  EXPECT_EQ(blackboard->get<std::string>("foo_branch_a"), "value_a");
  EXPECT_EQ(blackboard->get<std::string>("foo_branch_b"), "value_b");

  // Intermediate remapping keys must not leak into the final blackboard.
  EXPECT_FALSE(blackboard->contains("foo_inner_a"));
  EXPECT_FALSE(blackboard->contains("foo_inner_b"));

  // The original source key must also not exist in the final blackboard.
  EXPECT_FALSE(blackboard->contains("foo_str"));
}
