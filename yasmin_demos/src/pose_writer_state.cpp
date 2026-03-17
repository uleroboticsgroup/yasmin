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

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/pose_writer_state.h"
#include "yasmin_ros/interface_serialization.hpp"
#include "yasmin_ros/ros_logs.hpp"

PoseWriterState::PoseWriterState() : yasmin::State({"outcome1"}) {
  this->set_description(
      "Creates a Pose message, serializes it, and stores the serialized bytes "
      "and type information in the blackboard.");
  this->add_output_key(
      "pose_bytes",
      "Serialized Pose message stored as bytes in the blackboard.");
  this->add_output_key("pose_bytes__type",
                       "Type information for the serialized Pose message.");
}

std::string PoseWriterState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state POSE_WRITER");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  const std::vector<uint8_t> pose_bytes =
      yasmin_ros::serialize_interface<geometry_msgs::msg::Pose>(pose);

  blackboard->set<std::vector<uint8_t>>("pose_bytes", pose_bytes);
  blackboard->set<std::string>("pose_bytes__type", "geometry_msgs/msg/Pose");

  YASMIN_LOG_INFO("Stored serialized Pose with %zu bytes", pose_bytes.size());

  return "outcome1";
}

PoseWriterState::~PoseWriterState() {}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PoseWriterState, yasmin::State)
