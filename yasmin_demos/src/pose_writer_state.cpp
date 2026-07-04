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

#include "yasmin_demos/pose_writer_state.hpp"

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/interface_serialization.hpp"
#include "yasmin_ros/ros_logs.hpp"

PoseWriterState::PoseWriterState() : yasmin::State({"outcome1"}) {
  this->position_x_ = 1.0;
  this->position_y_ = 2.0;
  this->position_z_ = 3.0;
  this->orientation_w_ = 1.0;
  this->set_description(
      "Creates a Pose message, serializes it, and stores the serialized bytes "
      "and type information in the blackboard.");
  this->declare_parameter<double>("position_x", "Pose position x coordinate.",
                                  1.0);
  this->declare_parameter<double>("position_y", "Pose position y coordinate.",
                                  2.0);
  this->declare_parameter<double>("position_z", "Pose position z coordinate.",
                                  3.0);
  this->declare_parameter<double>("orientation_w",
                                  "Pose orientation w component.", 1.0);
  this->add_output_key(
      "pose_bytes",
      "Serialized Pose message stored as bytes in the blackboard.");
  this->add_output_key("pose_bytes__type",
                       "Type information for the serialized Pose message.");
}

void PoseWriterState::configure() {
  this->position_x_ = this->get_parameter<double>("position_x");
  this->position_y_ = this->get_parameter<double>("position_y");
  this->position_z_ = this->get_parameter<double>("position_z");
  this->orientation_w_ = this->get_parameter<double>("orientation_w");
}

std::string PoseWriterState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state POSE_WRITER");

  geometry_msgs::msg::Pose pose;
  pose.position.x = this->position_x_;
  pose.position.y = this->position_y_;
  pose.position.z = this->position_z_;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = this->orientation_w_;

  const std::vector<uint8_t> pose_bytes =
      yasmin_ros::serialize_interface<geometry_msgs::msg::Pose>(pose);

  blackboard->set<std::vector<uint8_t>>("pose_bytes", pose_bytes);
  blackboard->set<std::string>("pose_bytes__type", "geometry_msgs/msg/Pose");

  YASMIN_LOG_INFO("Stored serialized Pose with %zu bytes", pose_bytes.size());

  return "outcome1";
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PoseWriterState, yasmin::State)
