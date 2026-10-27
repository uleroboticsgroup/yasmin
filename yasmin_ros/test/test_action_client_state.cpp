// Copyright (C) 2025 Miguel Ángel González Santamarta
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
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "yasmin/types.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;
using namespace std::chrono_literals;

class AuxNode : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  AuxNode() : Node("test_node") {
    // Action server
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "test",
        std::bind(&AuxNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&AuxNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&AuxNode::handle_accepted, this, std::placeholders::_1));
  }

  void set_goal_response_delay(std::chrono::milliseconds delay) {
    this->goal_response_delay_ms_.store(delay.count());
  }

  void set_execution_delay(std::chrono::milliseconds delay) {
    this->execution_delay_ms_.store(delay.count());
  }

  void reset_flags() {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->goal_received_ = false;
    this->cancel_received_ = false;
  }

  bool wait_for_goal_received(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(this->state_mutex_);
    return this->state_cv_.wait_for(lock, timeout,
                                    [this]() { return this->goal_received_; });
  }

  bool wait_for_cancel_received(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(this->state_mutex_);
    return this->state_cv_.wait_for(
        lock, timeout, [this]() { return this->cancel_received_; });
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  std::atomic<int64_t> goal_response_delay_ms_{0};
  std::atomic<int64_t> execution_delay_ms_{5000};
  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  bool goal_received_{false};
  bool cancel_received_{false};

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Fibonacci::Goal> goal) {
    (void)uuid;
    (void)goal;

    {
      std::lock_guard<std::mutex> lock(this->state_mutex_);
      this->goal_received_ = true;
    }
    this->state_cv_.notify_all();

    const int64_t delay_ms = this->goal_response_delay_ms_.load();
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    (void)goal_handle;
    {
      std::lock_guard<std::mutex> lock(this->state_mutex_);
      this->cancel_received_ = true;
    }
    this->state_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    std::thread{
        std::bind(&AuxNode::execute_action, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  void execute_action(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Fibonacci::Result>();

    if (goal->order < 0) {
      goal_handle->abort(result);
    } else {
      const int64_t delay_ms = this->execution_delay_ms_.load();
      if (delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
      }
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->succeed(result);
      }
    }
  }
};

class TestActionClientState : public ::testing::Test {
protected:
  static std::shared_ptr<AuxNode> aux_node;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  static std::thread spin_thread;

  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    aux_node = std::make_shared<AuxNode>();
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(aux_node);
    spin_thread = std::thread([&]() { executor->spin(); });
  }

  void SetUp() override {
    aux_node->set_goal_response_delay(0ms);
    aux_node->set_execution_delay(5s);
    aux_node->reset_flags();
  }

  static void TearDownTestCase() {
    executor->cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    aux_node.reset();
    executor.reset();
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
    rclcpp::shutdown();
  }
};

std::shared_ptr<AuxNode> TestActionClientState::aux_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    TestActionClientState::executor = nullptr;
std::thread TestActionClientState::spin_thread;

TEST_F(TestActionClientState, TestActionClientStateSucceed) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  EXPECT_EQ((*state)(blackboard), std::string(SUCCEED));
}

TEST_F(TestActionClientState, TestActionClientStateCache) {
  ROSClientsCache::clear_all();
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 0);

  auto state1 =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 1);

  auto state2 =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 1);

  auto state3 =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test2", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 2);
}

TEST_F(TestActionClientState, TestActionClientStateResultHandler) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          yasmin::Outcomes{"new_outcome"},
          [](yasmin::Blackboard::SharedPtr blackboard,
             std::shared_ptr<example_interfaces::action::Fibonacci::Result>
                 result) { return "new_outcome"; });

  EXPECT_EQ((*state)(blackboard), "new_outcome");
}

TEST_F(TestActionClientState, TestActionClientStateCancel) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 20;
            return goal;
          });

  std::thread cancel_thread([&state]() {
    std::this_thread::sleep_for(1s);
    state->cancel_state();
  });

  EXPECT_EQ((*state)(blackboard), std::string(CANCEL));
  cancel_thread.join();
}

TEST_F(TestActionClientState, TestActionClientStateCancelBeforeGoalResponse) {
  aux_node->set_goal_response_delay(1s);
  aux_node->set_execution_delay(2s);

  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  std::string outcome;
  std::thread state_thread(
      [&state, &blackboard, &outcome]() { outcome = (*state)(blackboard); });

  ASSERT_TRUE(aux_node->wait_for_goal_received(5s));
  state->cancel_state();
  state_thread.join();

  EXPECT_EQ(outcome, std::string(CANCEL));
  EXPECT_TRUE(aux_node->wait_for_cancel_received(2s));
}

TEST_F(TestActionClientState,
       TestActionClientStateCancelBeforeGoalResponseReuse) {
  aux_node->set_goal_response_delay(1s);
  aux_node->set_execution_delay(2s);

  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  std::string first_outcome;
  std::thread first_thread([&state, &blackboard, &first_outcome]() {
    first_outcome = (*state)(blackboard);
  });

  ASSERT_TRUE(aux_node->wait_for_goal_received(5s));
  state->cancel_state();
  first_thread.join();
  EXPECT_EQ(first_outcome, std::string(CANCEL));

  std::string second_outcome;
  std::thread second_thread([&state, &blackboard, &second_outcome]() {
    second_outcome = (*state)(blackboard);
  });
  second_thread.join();

  EXPECT_EQ(second_outcome, std::string(SUCCEED));
  EXPECT_TRUE(aux_node->wait_for_cancel_received(2s));
}

TEST_F(TestActionClientState, TestActionClientStateAbort) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = -1;
            return goal;
          });

  EXPECT_EQ((*state)(blackboard), std::string(ABORT));
}

TEST_F(TestActionClientState, TestActionClientStateRetryWaitTimeout) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test1",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          },
          yasmin::Outcomes{}, 1, -1, 3); // wait_timeout=1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

TEST_F(TestActionClientState, TestActionClientStateRetryResponseTimeout) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          },
          yasmin::Outcomes{}, -1, 1,
          3); // response_timeout=1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
