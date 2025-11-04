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
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(TestPublisherState, TestPublisher) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
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
      "test", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 1);

  auto state2 = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto msg = std_msgs::msg::String();
        msg.data = "data";
        return msg;
      });
  EXPECT_EQ(ROSClientsCache::get_publishers_count(), 1);

  auto state3 = std::make_shared<PublisherState<std_msgs::msg::String>>(
      "test2", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
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
