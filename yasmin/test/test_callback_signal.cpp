// Copyright (C) 2026 Maik Knof
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

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "yasmin/blackboard.hpp"
#include "yasmin/callback_signal.hpp"
#include "yasmin/cb_state.hpp"

using namespace yasmin;

TEST(TestCallbackSignal, TestTriggerRunsAllCallbacks) {
  CallbackSignal signal;
  std::vector<int> execution_order;

  signal.add_callback([&execution_order]() { execution_order.push_back(1); });
  signal.add_callback([&execution_order]() { execution_order.push_back(2); });

  signal.trigger();

  ASSERT_EQ(execution_order.size(), 2U);
  EXPECT_EQ(execution_order.at(0), 1);
  EXPECT_EQ(execution_order.at(1), 2);
}

TEST(TestCallbackSignal, TestRemoveCallback) {
  CallbackSignal signal;
  std::atomic<int> count{0};

  const auto first_id = signal.add_callback([&count]() { count.fetch_add(1); });
  signal.add_callback([&count]() { count.fetch_add(1); });

  EXPECT_TRUE(signal.remove_callback(first_id));
  EXPECT_FALSE(signal.remove_callback(first_id));

  signal.trigger();

  EXPECT_EQ(count.load(), 1);
}

TEST(TestCallbackSignal, TestTriggerAsyncReturnsFuture) {
  CallbackSignal signal;
  std::atomic<bool> finished{false};

  signal.add_callback([&finished]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    finished.store(true);
  });

  auto future = signal.trigger_async();

  EXPECT_FALSE(future->is_completed());
  future->wait();
  EXPECT_TRUE(future->is_completed());
  EXPECT_TRUE(finished.load());
  EXPECT_FALSE(future->has_exception());
}

TEST(TestCallbackSignal, TestTriggerPropagatesFirstExceptionAfterDispatch) {
  CallbackSignal signal;
  std::atomic<int> count{0};

  signal.add_callback([&count]() { count.fetch_add(1); });
  signal.add_callback([]() { throw std::runtime_error("callback failure"); });
  signal.add_callback([&count]() { count.fetch_add(1); });

  EXPECT_THROW(
      {
        try {
          signal.trigger();
        } catch (const std::runtime_error &error) {
          EXPECT_STREQ(error.what(), "callback failure");
          throw;
        }
      },
      std::runtime_error);

  EXPECT_EQ(count.load(), 2);
}

TEST(TestCallbackSignal, TestAddCancelCallbackCancelsState) {
  auto state = std::make_shared<CbState>(Outcomes{"done"},
                                         [](Blackboard::SharedPtr blackboard) {
                                           (void)blackboard;
                                           return std::string("done");
                                         });
  CallbackSignal signal;

  EXPECT_FALSE(state->is_canceled());
  signal.add_cancel_callback(state);
  signal.trigger();
  EXPECT_TRUE(state->is_canceled());
}

TEST(TestCallbackSignal, TestBlackboardStoresSharedSignal) {
  Blackboard blackboard;
  auto signal = CallbackSignal::make_shared();
  std::atomic<int> count{0};

  blackboard.set<CallbackSignal::SharedPtr>("signal", signal);

  auto retrieved_signal = blackboard.get<CallbackSignal::SharedPtr>("signal");
  retrieved_signal->add_callback([&count]() { count.fetch_add(1); });

  signal->trigger();
  EXPECT_EQ(count.load(), 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
