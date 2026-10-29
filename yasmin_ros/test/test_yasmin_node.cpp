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

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <tuple>

#include "yasmin_ros/yasmin_node.hpp"

// Foxy's bundled Google Test uses the legacy test-case macro.
#ifndef INSTANTIATE_TEST_SUITE_P
#define INSTANTIATE_TEST_SUITE_P INSTANTIATE_TEST_CASE_P
#endif

class TestYasminNode : public ::testing::Test {
protected:
  void SetUp() override {
    const char *value = std::getenv("YASMIN_USE_SIM_TIME");
    had_value_ = value != nullptr;
    if (had_value_) {
      original_value_ = value;
    }
    unsetenv("YASMIN_USE_SIM_TIME");
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
    yasmin_ros::YasminNode::destroy_instance();
    if (had_value_) {
      setenv("YASMIN_USE_SIM_TIME", original_value_.c_str(), 1);
    } else {
      unsetenv("YASMIN_USE_SIM_TIME");
    }
  }

private:
  bool had_value_ = false;
  std::string original_value_;
};

class TestYasminNodeEnvironment
    : public TestYasminNode,
      public ::testing::WithParamInterface<std::tuple<const char *, bool>> {};

TEST_P(TestYasminNodeEnvironment, SimTimeEnvironment) {
  const auto [value, expected] = GetParam();
  if (value != nullptr) {
    ASSERT_EQ(setenv("YASMIN_USE_SIM_TIME", value, 1), 0);
  }

  auto node = yasmin_ros::YasminNode::get_instance();
  EXPECT_EQ(node->get_parameter("use_sim_time").as_bool(), expected);
  EXPECT_EQ(node->get_clock()->ros_time_is_active(), expected);
}

INSTANTIATE_TEST_SUITE_P(SimTime, TestYasminNodeEnvironment,
                         ::testing::Values(std::make_tuple(nullptr, false),
                                           std::make_tuple("true", true),
                                           std::make_tuple("True", true),
                                           std::make_tuple("TRUE", true),
                                           std::make_tuple("1", true),
                                           std::make_tuple("false", false),
                                           std::make_tuple("False", false),
                                           std::make_tuple("0", false),
                                           std::make_tuple(" \tTrue\n", true)));

TEST_F(TestYasminNode, InvalidSimTimeEnvironment) {
  for (const char *value : {"", "yes", "2"}) {
    ASSERT_EQ(setenv("YASMIN_USE_SIM_TIME", value, 1), 0);
    EXPECT_THROW(yasmin_ros::YasminNode::get_instance(), std::invalid_argument);
  }
}
