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
#include <memory>
#include <thread>

#include "example_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Fibonacci::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    (void)goal_handle;
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
      std::this_thread::sleep_for(5s);
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

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    aux_node = std::make_shared<AuxNode>();
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(aux_node);
    spin_thread = std::thread([&]() { executor->spin(); });
  }

  static void TearDownTestSuite() {
    executor->cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    aux_node.reset();
    executor.reset();
    rclcpp::shutdown();
  }
};

std::shared_ptr<AuxNode> TestActionClientState::aux_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    TestActionClientState::executor = nullptr;
std::thread TestActionClientState::spin_thread;

TEST_F(TestActionClientState, TestActionClientStateSucceed) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
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
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 1);

  auto state2 =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 1);

  auto state3 =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test2",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });
  EXPECT_EQ(ROSClientsCache::get_action_clients_count(), 2);
}

TEST_F(TestActionClientState, TestActionClientStateResultHandler) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          std::set<std::string>{"new_outcome"},
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
             std::shared_ptr<example_interfaces::action::Fibonacci::Result>
                 result) { return "new_outcome"; });

  EXPECT_EQ((*state)(blackboard), "new_outcome");
}

TEST_F(TestActionClientState, TestActionClientStateCancel) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
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

TEST_F(TestActionClientState, TestActionClientStateAbort) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = -1;
            return goal;
          });

  EXPECT_EQ((*state)(blackboard), std::string(ABORT));
}

TEST_F(TestActionClientState, TestActionClientStateRetryWaitTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test1",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          },
          std::set<std::string>{}, 1, -1, 3); // wait_timeout=1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

TEST_F(TestActionClientState, TestActionClientStateRetryResponseTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          },
          std::set<std::string>{}, -1, 1,
          3); // response_timeout=1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
