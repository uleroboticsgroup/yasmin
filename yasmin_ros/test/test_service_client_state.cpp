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

#include "example_interfaces/srv/add_two_ints.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;
using namespace std::chrono_literals;

class AuxNode : public rclcpp::Node {
public:
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  AuxNode() : Node("test_node") {
    // Service server
    this->service_server_ = this->create_service<AddTwoInts>(
        "test", std::bind(&AuxNode::handle_service, this, std::placeholders::_1,
                          std::placeholders::_2));
  }

private:
  rclcpp::Service<AddTwoInts>::SharedPtr service_server_;

  void handle_service(const std::shared_ptr<AddTwoInts::Request> request,
                      std::shared_ptr<AddTwoInts::Response> response) {
    std::this_thread::sleep_for(5s);
    response->sum = request->a + request->b;
  }
};

class TestServiceClientState : public ::testing::Test {
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

std::shared_ptr<AuxNode> TestServiceClientState::aux_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    TestServiceClientState::executor = nullptr;
std::thread TestServiceClientState::spin_thread;

TEST_F(TestServiceClientState, TestServiceClient) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 2;
            request->b = 3;
            return request;
          },
          -1, -1, 3); // wait_timeout, response_timeout, maximum_retry

  EXPECT_EQ((*state)(blackboard), std::string(SUCCEED));
}

TEST_F(TestServiceClientState, TestServiceClientCache) {
  ROSClientsCache::clear_all();
  EXPECT_EQ(ROSClientsCache::get_service_clients_count(), 0);

  auto state1 = std::make_shared<
      ServiceState<example_interfaces::srv::AddTwoInts>>(
      "test", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        return request;
      });
  EXPECT_EQ(ROSClientsCache::get_service_clients_count(), 1);

  auto state2 = std::make_shared<
      ServiceState<example_interfaces::srv::AddTwoInts>>(
      "test", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        return request;
      });
  EXPECT_EQ(ROSClientsCache::get_service_clients_count(), 1);

  auto state3 = std::make_shared<
      ServiceState<example_interfaces::srv::AddTwoInts>>(
      "test2", [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        return request;
      });
  EXPECT_EQ(ROSClientsCache::get_service_clients_count(), 2);
}

TEST_F(TestServiceClientState, TestServiceClientResponseHandler) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 2;
            request->b = 3;
            return request;
          },
          std::set<std::string>{"new_outcome"},
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>
                 response) { return "new_outcome"; });

  EXPECT_EQ((*state)(blackboard), "new_outcome");
}

TEST_F(TestServiceClientState, TestServiceClientRetryWaitTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test_retry",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 2;
            request->b = 3;
            return request;
          },
          1, -1, 3); // wait_timeout=1, response_timeout=-1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

TEST_F(TestServiceClientState, TestServiceClientRetryResponseTimeout) {
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

  auto state =
      std::make_shared<ServiceState<example_interfaces::srv::AddTwoInts>>(
          "test",
          [](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
            auto request = std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();
            request->a = 2;
            request->b = 3;
            return request;
          },
          -1, 1, 3); // wait_timeout=-1, response_timeout=1, maximum_retry=3

  EXPECT_EQ((*state)(blackboard), std::string(TIMEOUT));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
