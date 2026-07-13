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

#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/join_state.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Auxiliary node providing action server, service server, and publisher
// ---------------------------------------------------------------------------
class AuxNode : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  AuxNode() : Node("test_complex_node") {
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "test_action",
        std::bind(&AuxNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&AuxNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&AuxNode::handle_accepted, this, std::placeholders::_1));

    this->srv_ = this->create_service<AddTwoInts>(
        "test_service",
        std::bind(&AuxNode::handle_service, this, std::placeholders::_1,
                  std::placeholders::_2));

    this->publisher_ =
        this->create_publisher<std_msgs::msg::String>("test_topic", 10);
    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&AuxNode::publish_monitor_msg, this));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int monitor_msg_count_{0};

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
      std::this_thread::sleep_for(goal->order == 0 ? 2s : 1s);
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->succeed(result);
      }
    }
  }

  void handle_service(const AddTwoInts::Request::SharedPtr request,
                      AddTwoInts::Response::SharedPtr response) {
    response->sum = request->a + request->b;
  }

  void publish_monitor_msg() {
    auto msg = std_msgs::msg::String();
    msg.data = "monitor_data_" + std::to_string(monitor_msg_count_++);
    publisher_->publish(msg);
  }
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class TestComplexRosScenarios : public ::testing::Test {
protected:
  static std::shared_ptr<AuxNode> aux_node;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  static std::thread spin_thread;
  yasmin::Blackboard::SharedPtr bb;

  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    yasmin_ros::set_ros_loggers();
    aux_node = std::make_shared<AuxNode>();
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(aux_node);
    spin_thread = std::thread([&]() { executor->spin(); });
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

  void SetUp() override { bb = yasmin::Blackboard::make_shared(); }
};

std::shared_ptr<AuxNode> TestComplexRosScenarios::aux_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    TestComplexRosScenarios::executor = nullptr;
std::thread TestComplexRosScenarios::spin_thread;

// ---------------------------------------------------------------------------
// StateMachine chaining ROS states with CbState
// ---------------------------------------------------------------------------
TEST_F(TestComplexRosScenarios, TestSmChainActionCbService) {
  // ActionState -> CbState -> ServiceState with blackboard data flow
  auto action_state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          },
          yasmin::Outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT},
          [](yasmin::Blackboard::SharedPtr blackboard,
             std::shared_ptr<example_interfaces::action::Fibonacci::Result>
                 result) {
            (void)result;
            blackboard->set<int>("action_sum", 42);
            return std::string(SUCCEED);
          });

  auto cb_state = std::make_shared<yasmin::CbState>(
      yasmin::Outcomes{"proceed"},
      [](yasmin::Blackboard::SharedPtr blackboard) {
        int val = blackboard->get<int>("action_sum");
        blackboard->set<int>("service_input_a", val);
        blackboard->set<int>("service_input_b", val * 2);
        return std::string("proceed");
      });

  auto service_state = std::make_shared<
      ServiceState<example_interfaces::srv::AddTwoInts>>(
      "test_service",
      [](yasmin::Blackboard::SharedPtr blackboard) {
        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = blackboard->get<int>("service_input_a");
        request->b = blackboard->get<int>("service_input_b");
        return request;
      },
      yasmin::Outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT},
      [](yasmin::Blackboard::SharedPtr blackboard,
         example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
        blackboard->set<int>("final_sum", response->sum);
        return std::string(SUCCEED);
      });

  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::Outcomes{SUCCEED, "aborted", "canceled", "timeout"});
  sm->add_state("FETCH_ACTION", action_state,
                yasmin::Transitions{{SUCCEED, "PROCESS"},
                                    {ABORT, "aborted"},
                                    {CANCEL, "canceled"},
                                    {TIMEOUT, "timeout"}});
  sm->add_state("PROCESS", cb_state,
                yasmin::Transitions{{"proceed", "SEND_SERVICE"}});
  sm->add_state("SEND_SERVICE", service_state,
                yasmin::Transitions{{SUCCEED, SUCCEED},
                                    {ABORT, "aborted"},
                                    {CANCEL, "canceled"},
                                    {TIMEOUT, "timeout"}});

  std::string outcome = (*sm)(bb);
  EXPECT_EQ(outcome, SUCCEED);
  EXPECT_TRUE(bb->contains("final_sum"));
  EXPECT_EQ(bb->get<int>("final_sum"), 126);
}

TEST_F(TestComplexRosScenarios, TestSmChainActionPublisher) {
  // ActionState -> PublisherState
  auto action_state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          yasmin::Outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT},
          [](yasmin::Blackboard::SharedPtr blackboard,
             std::shared_ptr<example_interfaces::action::Fibonacci::Result>
                 result) {
            (void)result;
            blackboard->set<std::string>("publish_data", "action_done");
            return std::string(SUCCEED);
          });

  auto pub_state = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "pub_topic", [](yasmin::Blackboard::SharedPtr blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = blackboard->get<std::string>("publish_data");
        return msg;
      });

  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::Outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT});
  sm->add_state("DO_ACTION", action_state,
                yasmin::Transitions{{SUCCEED, "PUBLISH"},
                                    {ABORT, ABORT},
                                    {CANCEL, CANCEL},
                                    {TIMEOUT, TIMEOUT}});
  sm->add_state("PUBLISH", pub_state, yasmin::Transitions{{SUCCEED, SUCCEED}});

  std::string outcome = (*sm)(bb);
  EXPECT_EQ(outcome, SUCCEED);
  EXPECT_EQ(bb->get<std::string>("publish_data"), "action_done");
}

// ---------------------------------------------------------------------------
// Concurrence with ROS states
// ---------------------------------------------------------------------------
TEST_F(TestComplexRosScenarios, TestConcurrenceActionAndMonitor) {
  auto action =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 3;
            return goal;
          });

  auto monitor = std::make_shared<MonitorState<std_msgs::msg::String>>(
      "test_topic", yasmin::Outcomes{SUCCEED},
      [](yasmin::Blackboard::SharedPtr blackboard,
         std::shared_ptr<std_msgs::msg::String> msg) {
        blackboard->set<std::string>("monitor_received", msg->data);
        return std::string(SUCCEED);
      },
      rclcpp::QoS(10), 10, 5);

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"ACTION", action}, {"MONITOR", monitor}}, TIMEOUT,
      yasmin::OutcomeMap{
          {SUCCEED, {{"ACTION", SUCCEED}, {"MONITOR", SUCCEED}}}});

  std::string outcome = (*conc)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

TEST_F(TestComplexRosScenarios, TestConcurrenceActionAndService) {
  auto action =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  auto service =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 10;
            request->b = 20;
            return request;
          });

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"ACTION", action}, {"SERVICE", service}}, TIMEOUT,
      yasmin::OutcomeMap{
          {SUCCEED, {{"ACTION", SUCCEED}, {"SERVICE", SUCCEED}}}});

  std::string outcome = (*conc)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

TEST_F(TestComplexRosScenarios, TestConcurrenceWithCbAndAction) {
  auto action =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  auto cb_state = std::make_shared<yasmin::CbState>(
      yasmin::Outcomes{"cb_done"},
      [](yasmin::Blackboard::SharedPtr blackboard) {
        blackboard->set<bool>("cb_ran", true);
        return std::string("cb_done");
      });

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"ACTION", action}, {"CB", cb_state}}, TIMEOUT,
      yasmin::OutcomeMap{{SUCCEED, {{"ACTION", SUCCEED}, {"CB", "cb_done"}}}});

  std::string outcome = (*conc)(bb);
  EXPECT_EQ(outcome, SUCCEED);
  EXPECT_TRUE(bb->get<bool>("cb_ran"));
}

TEST_F(TestComplexRosScenarios, TestConcurrenceMonitorAndService) {
  auto monitor = std::make_shared<MonitorState<std_msgs::msg::String>>(
      "test_topic", yasmin::Outcomes{SUCCEED},
      [](yasmin::Blackboard::SharedPtr blackboard,
         std::shared_ptr<std_msgs::msg::String> msg) {
        (void)blackboard;
        (void)msg;
        return std::string(SUCCEED);
      },
      rclcpp::QoS(10), 10, 5);

  auto service =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 5;
            request->b = 7;
            return request;
          });

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"MONITOR", monitor}, {"SERVICE", service}}, TIMEOUT,
      yasmin::OutcomeMap{
          {SUCCEED, {{"MONITOR", SUCCEED}, {"SERVICE", SUCCEED}}}});

  std::string outcome = (*conc)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

TEST_F(TestComplexRosScenarios, TestConcurrenceMixedOutcomeMapping) {
  auto monitor = std::make_shared<MonitorState<std_msgs::msg::String>>(
      "test_topic", yasmin::Outcomes{"special_outcome"},
      [](yasmin::Blackboard::SharedPtr blackboard,
         std::shared_ptr<std_msgs::msg::String> msg) {
        (void)blackboard;
        (void)msg;
        return std::string("special_outcome");
      },
      rclcpp::QoS(10), 10, 5);

  auto service =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 3;
            request->b = 4;
            return request;
          });

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"MONITOR", monitor}, {"SERVICE", service}}, TIMEOUT,
      yasmin::OutcomeMap{
          {"custom_done",
           {{"MONITOR", "special_outcome"}, {"SERVICE", SUCCEED}}}});

  std::string outcome = (*conc)(bb);
  EXPECT_EQ(outcome, "custom_done");
}

// ---------------------------------------------------------------------------
// OrthogonalState with ROS states
// ---------------------------------------------------------------------------

TEST_F(TestComplexRosScenarios, TestOrthogonalWithRosRegions) {
  yasmin::Outcomes ros_outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT};

  auto region_a = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_a->set_name("RegionA");
  region_a->add_state(
      "ACTION",
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, SUCCEED},
                          {ABORT, ABORT},
                          {CANCEL, CANCEL},
                          {TIMEOUT, TIMEOUT}});

  auto region_b = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_b->set_name("RegionB");
  region_b->add_state(
      "SERVICE",
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 7;
            request->b = 8;
            return request;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, SUCCEED},
                          {ABORT, ABORT},
                          {CANCEL, CANCEL},
                          {TIMEOUT, TIMEOUT}});

  auto ort = std::make_shared<yasmin::OrthogonalState>(
      TIMEOUT, yasmin::OutcomeMap{
                   {SUCCEED, {{"RegionA", SUCCEED}, {"RegionB", SUCCEED}}}});
  ort->add_region("RegionA", region_a);
  ort->add_region("RegionB", region_b);
  ort->configure();

  std::string outcome = (*ort)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

TEST_F(TestComplexRosScenarios, TestOrthogonalWithJoinAndRos) {
  yasmin::Outcomes ros_outcomes{SUCCEED};

  auto region_a = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_a->set_name("RegionA");
  region_a->add_state(
      "ACTION",
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, "JOIN"}});
  region_a->add_state("JOIN", std::make_shared<yasmin::JoinState>("sync_point"),
                      yasmin::Transitions{{"joined", "DONE"}});
  region_a->add_state("DONE",
                      std::make_shared<yasmin::CbState>(
                          ros_outcomes,
                          [](yasmin::Blackboard::SharedPtr blackboard) {
                            (void)blackboard;
                            return std::string(SUCCEED);
                          }),
                      yasmin::Transitions{{SUCCEED, SUCCEED}});

  auto region_b = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_b->set_name("RegionB");
  region_b->add_state(
      "SERVICE",
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 10;
            request->b = 20;
            return request;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, "JOIN"}});
  region_b->add_state("JOIN", std::make_shared<yasmin::JoinState>("sync_point"),
                      yasmin::Transitions{{"joined", "DONE"}});
  region_b->add_state("DONE",
                      std::make_shared<yasmin::CbState>(
                          ros_outcomes,
                          [](yasmin::Blackboard::SharedPtr blackboard) {
                            (void)blackboard;
                            return std::string(SUCCEED);
                          }),
                      yasmin::Transitions{{SUCCEED, SUCCEED}});

  auto ort = std::make_shared<yasmin::OrthogonalState>(
      TIMEOUT, yasmin::OutcomeMap{
                   {SUCCEED, {{"RegionA", SUCCEED}, {"RegionB", SUCCEED}}}});
  ort->add_region("RegionA", region_a);
  ort->add_region("RegionB", region_b);
  ort->configure();

  std::string outcome = (*ort)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

// ---------------------------------------------------------------------------
// StateMachine with nested Concurrence / OrthogonalState
// ---------------------------------------------------------------------------
TEST_F(TestComplexRosScenarios, TestSmWithNestedConcurrenceOfRosStates) {
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::Outcomes{SUCCEED, TIMEOUT});

  sm->add_state("START",
                std::make_shared<yasmin::CbState>(
                    yasmin::Outcomes{"go_next"},
                    [](yasmin::Blackboard::SharedPtr blackboard) {
                      (void)blackboard;
                      return std::string("go_next");
                    }),
                yasmin::Transitions{{"go_next", "PARALLEL"}});

  auto action =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          });

  auto service =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service", [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 3;
            request->b = 4;
            return request;
          });

  auto conc = yasmin::Concurrence::make_shared(
      yasmin::StateMap{{"ACTION", action}, {"SERVICE", service}}, TIMEOUT,
      yasmin::OutcomeMap{
          {SUCCEED, {{"ACTION", SUCCEED}, {"SERVICE", SUCCEED}}}});

  sm->add_state("PARALLEL", conc,
                yasmin::Transitions{{SUCCEED, SUCCEED}, {TIMEOUT, TIMEOUT}});

  std::string outcome = (*sm)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

TEST_F(TestComplexRosScenarios, TestSmWithOrthogonalOfRosRegions) {
  yasmin::Outcomes ros_outcomes{SUCCEED};

  auto region_a = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_a->set_name("RegionA");
  region_a->add_state(
      "ACTION",
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, SUCCEED}});

  auto region_b = std::make_shared<yasmin::StateMachine>(ros_outcomes);
  region_b->set_name("RegionB");
  region_b->add_state(
      "SERVICE",
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_service",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 5;
            request->b = 6;
            return request;
          },
          ros_outcomes, -1),
      yasmin::Transitions{{SUCCEED, SUCCEED}});

  auto ort = std::make_shared<yasmin::OrthogonalState>(
      TIMEOUT, yasmin::OutcomeMap{
                   {SUCCEED, {{"RegionA", SUCCEED}, {"RegionB", SUCCEED}}}});
  ort->add_region("RegionA", region_a);
  ort->add_region("RegionB", region_b);
  ort->configure();

  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::Outcomes{SUCCEED, TIMEOUT});
  sm->add_state("START",
                std::make_shared<yasmin::CbState>(
                    yasmin::Outcomes{"go"},
                    [](yasmin::Blackboard::SharedPtr blackboard) {
                      (void)blackboard;
                      return std::string("go");
                    }),
                yasmin::Transitions{{"go", "ORTHO"}});
  sm->add_state("ORTHO", ort,
                yasmin::Transitions{{SUCCEED, SUCCEED}, {TIMEOUT, TIMEOUT}});

  std::string outcome = (*sm)(bb);
  EXPECT_EQ(outcome, SUCCEED);
}

// ---------------------------------------------------------------------------
// Blackboard data flow through sequential StateMachine
// ---------------------------------------------------------------------------
TEST_F(TestComplexRosScenarios, TestBlackboardFlowThroughStateMachine) {
  auto action_state =
      std::make_shared<ActionState<example_interfaces::action::Fibonacci>>(
          "test_action",
          [](yasmin::Blackboard::SharedPtr blackboard) {
            auto goal = example_interfaces::action::Fibonacci::Goal();
            goal.order = 0;
            return goal;
          },
          yasmin::Outcomes{SUCCEED, ABORT, CANCEL, TIMEOUT},
          [](yasmin::Blackboard::SharedPtr blackboard,
             std::shared_ptr<example_interfaces::action::Fibonacci::Result>
                 result) {
            (void)result;
            blackboard->set<std::string>("action_result", "from_action");
            return std::string(SUCCEED);
          });

  auto check_bb_state = std::make_shared<yasmin::CbState>(
      yasmin::Outcomes{"checked"},
      [](yasmin::Blackboard::SharedPtr blackboard) {
        bool passed = blackboard->contains("action_result");
        blackboard->set<bool>("check_passed", passed);
        return std::string("checked");
      });

  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::Outcomes{SUCCEED, "checked"});
  sm->add_state("ACTION", action_state,
                yasmin::Transitions{{SUCCEED, "CHECK"},
                                    {ABORT, SUCCEED},
                                    {CANCEL, SUCCEED},
                                    {TIMEOUT, SUCCEED}});
  sm->add_state("CHECK", check_bb_state,
                yasmin::Transitions{{"checked", SUCCEED}});

  std::string outcome = (*sm)(bb);
  EXPECT_EQ(outcome, SUCCEED);
  EXPECT_TRUE(bb->get<bool>("check_passed"));
  EXPECT_EQ(bb->get<std::string>("action_result"), "from_action");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
