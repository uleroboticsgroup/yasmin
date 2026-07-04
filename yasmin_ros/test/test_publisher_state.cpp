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

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "yasmin/types.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;
using namespace std::chrono_literals;

class TestPublisherState : public ::testing::Test {
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() {
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
    rclcpp::shutdown();
  }
};

TEST_F(TestPublisherState, TestPublisher) {
  auto blackboard = yasmin::Blackboard::make_shared();

  auto state = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test", [](yasmin::Blackboard::SharedPtr blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });

  EXPECT_EQ((*state)(blackboard), std::string(SUCCEED));
}

TEST_F(TestPublisherState, TestPublisherCache) {
  ROSClientsCache::clear_all();
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 0);

  auto state1 = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test", [](yasmin::Blackboard::SharedPtr blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 1);

  auto state2 = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test", [](yasmin::Blackboard::SharedPtr blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 1);

  auto state3 = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test2", [](yasmin::Blackboard::SharedPtr blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
