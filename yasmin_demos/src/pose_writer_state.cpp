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
  position_x_ = 1.0;
  position_y_ = 2.0;
  position_z_ = 3.0;
  orientation_w_ = 1.0;
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
  position_x_ = this->get_parameter<double>("position_x");
  position_y_ = this->get_parameter<double>("position_y");
  position_z_ = this->get_parameter<double>("position_z");
  orientation_w_ = this->get_parameter<double>("orientation_w");
}

std::string PoseWriterState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state POSE_WRITER");

  geometry_msgs::msg::Pose pose;
  pose.position.x = position_x_;
  pose.position.y = position_y_;
  pose.position.z = position_z_;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = orientation_w_;

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
