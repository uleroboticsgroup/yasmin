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

#include "yasmin_pcl/io/ros_to_pcl_point_cloud2_state.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <exception>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

RosToPclPointCloud2State::RosToPclPointCloud2State()
    : yasmin::State({"succeeded", "aborted"}) {
  this->set_description(
      "Converts a ROS sensor_msgs::msg::PointCloud2 message from the "
      "blackboard into pcl::PCLPointCloud2.");
  this->set_outcome_description("succeeded",
                                "The ROS point cloud was converted to PCL.");
  this->set_outcome_description("aborted",
                                "The input cloud was missing or invalid.");
  this->add_input_key(
      "input_cloud",
      "Input cloud stored as std::shared_ptr<sensor_msgs::msg::PointCloud2>.");
  this->add_output_key("output_cloud",
                       "Converted cloud stored as pcl::PCLPointCloud2::Ptr.");
}

std::string
RosToPclPointCloud2State::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::RosPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input ROS point cloud pointer is null");
      return "aborted";
    }

    auto output_cloud = common::make_pcl_point_cloud2();
    pcl_conversions::toPCL(*input_cloud, *output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to convert ROS point cloud to PCL: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::RosToPclPointCloud2State, yasmin::State)
