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

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;
using namespace std::chrono_literals;

class AuxNode : public rclcpp::Node {
public:
  AuxNode() : Node("test_node") {
    // Publisher
    this->publisher_ =
        this->create_publisher<std_msgs::msg::String>("test", 10);
    this->timer_ =
        this->create_wall_timer(1s, std::bind(&AuxNode::publish_message, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_message() {
    if (rclcpp::ok()) {
      auto message = std_msgs::msg::String();
      message.data = "data";
      publisher_->publish(message);
    }
  }
};

class TestMonitorState : public ::testing::Test {
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
    ROSClientsCache::clear_all();
    aux_node.reset();
    executor.reset();
    rclcpp::shutdown();
  }
};

std::shared_ptr<AuxNode> TestMonitorState::aux_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    TestMonitorState::executor = nullptr;
std::thread TestMonitorState::spin_thread;

TEST_F(TestMonitorState, TestMonitorTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state = std::make_shared<MonitorState<std_msgs::msg::String>>(
      "test1", std::set<std::string>{std::string(SUCCEED)},
      [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
         std::shared_ptr<std_msgs::msg::String> msg) {
        return std::string(SUCCEED);
      },
      rclcpp::QoS(10), 10, 2);

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

TEST_F(TestMonitorState, TestMonitorRetryTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state = std::make_shared<MonitorState<std_msgs::msg::String>>(
      "test1", std::set<std::string>{std::string(SUCCEED)},
      [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
         std::shared_ptr<std_msgs::msg::String> msg) {
        return std::string(SUCCEED);
      },
      rclcpp::QoS(10), 10, 2, 3); // timeout=2, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
