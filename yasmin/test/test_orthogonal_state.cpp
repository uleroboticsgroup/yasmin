// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "yasmin/blackboard.hpp"
#include "yasmin/join_state.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/region_barrier.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

// ---------------------------------------------------------------------------
// Test states
// ---------------------------------------------------------------------------
class TestStateA : public State {
public:
  TestStateA() : State({"done"}) {}
  std::string execute(Blackboard::SharedPtr) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return "done";
  }
};

class TestStateB : public State {
public:
  TestStateB() : State({"done"}) {}
  std::string execute(Blackboard::SharedPtr) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return "done";
  }
};

// ---------------------------------------------------------------------------
// RegionBarrier tests
// ---------------------------------------------------------------------------
TEST(TestRegionBarrier, TestBasicSync) {
  auto barrier = std::make_shared<RegionBarrier>(2);
  std::atomic<int> counter{0};

  std::thread t1([barrier, &counter]() {
    counter++;
    barrier->arrive_and_wait();
    counter++;
  });

  std::thread t2([barrier, &counter]() {
    counter++;
    barrier->arrive_and_wait();
    counter++;
  });

  t1.join();
  t2.join();

  EXPECT_EQ(counter, 4);
}

TEST(TestRegionBarrier, TestCancel) {
  auto barrier = std::make_shared<RegionBarrier>(2);
  std::atomic<bool> released{false};

  std::thread t1([barrier, &released]() {
    barrier->arrive_and_wait();
    released = true;
    // After cancel, the barrier's canceled_ flag is set
    // The thread exits without the second participant arriving
  });

  // Let t1 start and wait, then cancel
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  barrier->cancel();

  t1.join();
  // cancel() should release the waiter, so released becomes true
  EXPECT_TRUE(released);
}

TEST(TestRegionBarrier, TestReset) {
  auto barrier = std::make_shared<RegionBarrier>(2);
  std::atomic<int> counter{0};

  // First cycle
  std::thread t1([barrier, &counter]() {
    barrier->arrive_and_wait();
    counter++;
  });
  std::thread t2([barrier, &counter]() {
    barrier->arrive_and_wait();
    counter++;
  });
  t1.join();
  t2.join();
  EXPECT_EQ(counter, 2);

  // Reset and repeat
  barrier->reset();
  std::thread t3([barrier, &counter]() {
    barrier->arrive_and_wait();
    counter++;
  });
  std::thread t4([barrier, &counter]() {
    barrier->arrive_and_wait();
    counter++;
  });
  t3.join();
  t4.join();
  EXPECT_EQ(counter, 4);
}

// ---------------------------------------------------------------------------
// JoinState tests
// ---------------------------------------------------------------------------
TEST(TestJoinState, TestConstruction) {
  auto js = std::make_shared<JoinState>("sync_1");
  EXPECT_EQ(js->get_sync_id(), "sync_1");
  EXPECT_FALSE(js->get_barrier());
}

TEST(TestJoinState, TestSetBarrier) {
  auto js = std::make_shared<JoinState>("sync_1");
  auto barrier = std::make_shared<RegionBarrier>(2);
  js->set_barrier(barrier);
  EXPECT_TRUE(js->get_barrier() == barrier);
}

TEST(TestJoinState, TestExecuteWithoutBarrier) {
  auto js = std::make_shared<JoinState>("sync_1", "joined");
  auto bb = Blackboard::make_shared();
  EXPECT_EQ((*js)(bb), "joined");
}

TEST(TestJoinState, TestStr) {
  auto js = std::make_shared<JoinState>("sync_1");
  EXPECT_TRUE(js->to_string().find("sync_1") != std::string::npos);
}

// ---------------------------------------------------------------------------
// OrthogonalState tests
// ---------------------------------------------------------------------------
class TestOrthogonalState : public ::testing::Test {
protected:
  Blackboard::SharedPtr blackboard;

  void SetUp() override { blackboard = Blackboard::make_shared(); }

  StateMachine::SharedPtr make_region(const std::string &name,
                                      State::SharedPtr state) {
    auto sm = std::make_shared<StateMachine>(Outcomes{"done"});
    sm->set_name(name);
    sm->add_state("work", std::move(state), {{"done", "done"}});
    sm->set_start_state("work");
    return sm;
  }

  StateMachine::SharedPtr make_synced_region(const std::string &name,
                                             const std::string &sync_id,
                                             State::SharedPtr post_sync_state) {
    auto sm = std::make_shared<StateMachine>(Outcomes{"done"});
    sm->set_name(name);
    sm->add_state("work", std::make_shared<TestStateA>(), {{"done", "sync"}});
    sm->add_state("sync", std::make_shared<JoinState>(sync_id),
                  {{"joined", "finish"}});
    sm->add_state("finish", std::move(post_sync_state), {{"done", "done"}});
    sm->set_start_state("work");
    return sm;
  }
};

TEST_F(TestOrthogonalState, TestBasicConcurrentRegions) {
  auto ort = std::make_shared<OrthogonalState>("timeout");
  ort->add_region("A", make_region("A", std::make_shared<TestStateA>()));
  ort->add_region("B", make_region("B", std::make_shared<TestStateB>()));

  ort->configure();
  EXPECT_EQ((*ort)(blackboard), "timeout");
}

TEST_F(TestOrthogonalState, TestOutcomeMap) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"success", {{"A", "done"}, {"B", "done"}}}});
  ort->add_region("A", make_region("A", std::make_shared<TestStateA>()));
  ort->add_region("B", make_region("B", std::make_shared<TestStateB>()));

  EXPECT_EQ((*ort)(blackboard), "success");
}

TEST_F(TestOrthogonalState, TestSyncBarrier) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"success", {{"A", "done"}, {"B", "done"}}}});

  auto post_sync = std::make_shared<TestStateA>();
  ort->add_region("A", make_synced_region("A", "sync1", post_sync));
  ort->add_region("B", make_synced_region("B", "sync1", post_sync));

  EXPECT_EQ((*ort)(blackboard), "success");
}

TEST_F(TestOrthogonalState, TestCancel) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"success", {{"A", "done"}, {"B", "done"}}}});
  ort->add_region("A", make_region("A", std::make_shared<TestStateA>()));
  ort->add_region("B", make_region("B", std::make_shared<TestStateB>()));

  EXPECT_FALSE(ort->is_canceled());
  ort->cancel_state();
  EXPECT_TRUE(ort->is_canceled());
}

TEST_F(TestOrthogonalState, TestConfigureRunsOnce) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"success", {{"A", "done"}, {"B", "done"}}}});
  ort->add_region("A", make_region("A", std::make_shared<TestStateA>()));
  ort->add_region("B", make_region("B", std::make_shared<TestStateB>()));

  ort->configure();
  ort->configure(); // second call should be no-op
  EXPECT_EQ((*ort)(blackboard), "success");
}

TEST_F(TestOrthogonalState, TestStr) {
  auto ort = std::make_shared<OrthogonalState>("timeout");
  ort->add_region("A", make_region("A", std::make_shared<TestStateA>()));
  ort->add_region("B", make_region("B", std::make_shared<TestStateB>()));

  std::string s = ort->to_string();
  EXPECT_TRUE(s.find("A") != std::string::npos);
  EXPECT_TRUE(s.find("B") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Validate: OrthogonalState recursively validates region StateMachines
// ---------------------------------------------------------------------------

// Helper: a valid region SM where state transitions cover all outcomes.
static StateMachine::SharedPtr make_valid_region_for_validate() {
  auto sm = std::make_shared<StateMachine>(Outcomes{"done"});
  sm->add_state("A", std::make_shared<TestStateA>(), {{"done", "done"}});
  return sm;
}

// Helper: a strict-invalid region SM – TestStateA's "done" outcome has a
// transition but a hypothetical extra outcome "extra" does not.
class TestStateWithExtraOutcome : public State {
public:
  TestStateWithExtraOutcome() : State({"done", "extra"}) {}
  std::string execute(Blackboard::SharedPtr) override { return "done"; }
};

static StateMachine::SharedPtr make_strict_invalid_region_for_validate() {
  auto sm = std::make_shared<StateMachine>(Outcomes{"done"});
  sm->add_state("WORK", std::make_shared<TestStateWithExtraOutcome>(),
                {{"done", "done"}}); // "extra" outcome has no transition
  return sm;
}

TEST_F(TestOrthogonalState, TestValidatePassesForValidRegions) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"done", {{"A", "done"}, {"B", "done"}}}});
  ort->add_region("A", make_valid_region_for_validate());
  ort->add_region("B", make_valid_region_for_validate());

  EXPECT_NO_THROW(ort->validate());
}

TEST_F(TestOrthogonalState, TestValidateThrowsWhenRegionIsStrictInvalid) {
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"done", {{"A", "done"}}}});
  ort->add_region("A", make_strict_invalid_region_for_validate());

  // strict_mode=true: "extra" outcome of WORK has no transition ->
  // runtime_error
  EXPECT_THROW(ort->validate(true), std::runtime_error);
}

TEST_F(TestOrthogonalState,
       TestValidatePassesWhenRegionIsOnlyNonStrictInvalid) {
  // Non-strict mode does not check for every outcome having a transition.
  auto ort = std::make_shared<OrthogonalState>(
      "timeout", OutcomeMap{{"done", {{"A", "done"}}}});
  ort->add_region("A", make_strict_invalid_region_for_validate());

  EXPECT_NO_THROW(ort->validate(false));
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
