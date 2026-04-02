// Copyright (C) 2026 Maik Knof
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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard.hpp"
#include "yasmin_ros/interface_serialization.hpp"
#include "yasmin_ros/ros_deserialize_cpp_state.hpp"
#include "yasmin_ros/ros_serialize_cpp_state.hpp"

class TestRosSerializationStates : public ::testing::Test {
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST_F(TestRosSerializationStates,
       TestSerializeConfigureInvalidInterfaceThrows) {
  auto state = std::make_shared<yasmin_ros::RosSerializeCppState>();
  state->set_parameter<std::string>("interface_type", "does_not_exist");

  EXPECT_THROW(state->configure(), std::invalid_argument);
}

TEST_F(TestRosSerializationStates, TestSerializePoseSucceeds) {
  auto blackboard = yasmin::Blackboard::make_shared();
  auto state = std::make_shared<yasmin_ros::RosSerializeCppState>();
  state->configure();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.25;
  pose.position.y = -2.5;
  pose.position.z = 3.75;
  pose.orientation.w = 0.5;
  blackboard->set<geometry_msgs::msg::Pose>("input", pose);

  EXPECT_EQ((*state)(blackboard), std::string("succeed"));

  const auto serialized_data = blackboard->get<std::vector<uint8_t>>("output");
  EXPECT_FALSE(serialized_data.empty());
}

TEST_F(TestRosSerializationStates,
       TestSerializeReturnsTypeErrorForWrongInputType) {
  auto blackboard = yasmin::Blackboard::make_shared();
  auto state = std::make_shared<yasmin_ros::RosSerializeCppState>();
  state->configure();

  std_msgs::msg::String msg;
  msg.data = "wrong type";
  blackboard->set<std_msgs::msg::String>("input", msg);

  EXPECT_EQ((*state)(blackboard), std::string("type_error"));
}

TEST_F(TestRosSerializationStates,
       TestDeserializeConfigureInvalidInterfaceThrows) {
  auto state = std::make_shared<yasmin_ros::RosDeserializeCppState>();
  state->set_parameter<std::string>("interface_type", "does_not_exist");

  EXPECT_THROW(state->configure(), std::invalid_argument);
}

TEST_F(TestRosSerializationStates, TestDeserializePoseSucceeds) {
  auto blackboard = yasmin::Blackboard::make_shared();
  auto state = std::make_shared<yasmin_ros::RosDeserializeCppState>();
  state->configure();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 4.0;
  pose.position.y = 5.0;
  pose.position.z = 6.0;
  pose.orientation.z = 0.25;
  pose.orientation.w = 0.75;

  const auto serialized_data =
      yasmin_ros::serialize_interface<geometry_msgs::msg::Pose>(pose);
  blackboard->set<std::vector<uint8_t>>("input", serialized_data);

  EXPECT_EQ((*state)(blackboard), std::string("succeed"));

  const auto output_pose = blackboard->get<geometry_msgs::msg::Pose>("output");
  EXPECT_DOUBLE_EQ(output_pose.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(output_pose.position.y, pose.position.y);
  EXPECT_DOUBLE_EQ(output_pose.position.z, pose.position.z);
  EXPECT_DOUBLE_EQ(output_pose.orientation.z, pose.orientation.z);
  EXPECT_DOUBLE_EQ(output_pose.orientation.w, pose.orientation.w);
}

TEST_F(TestRosSerializationStates,
       TestDeserializeReturnsTypeErrorForWrongInputType) {
  auto blackboard = yasmin::Blackboard::make_shared();
  auto state = std::make_shared<yasmin_ros::RosDeserializeCppState>();
  state->configure();

  blackboard->set<std::string>("input", "not bytes");

  EXPECT_EQ((*state)(blackboard), std::string("type_error"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
