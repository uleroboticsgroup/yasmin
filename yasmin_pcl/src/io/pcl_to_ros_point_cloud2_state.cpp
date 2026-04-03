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

#include "yasmin_pcl/io/pcl_to_ros_point_cloud2_state.hpp"

#include <exception>
#include <memory>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

PclToRosPointCloud2State::PclToRosPointCloud2State()
    : yasmin::State({"succeeded", "aborted"}) {
  this->set_description(
      "Converts a pcl::PCLPointCloud2 cloud from the blackboard into a ROS "
      "sensor_msgs::msg::PointCloud2 message.");
  this->set_outcome_description("succeeded",
                                "The PCL point cloud was converted to ROS.");
  this->set_outcome_description("aborted",
                                "The input cloud was missing or invalid.");
  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key("output_cloud",
                       "Converted cloud stored as "
                       "std::shared_ptr<sensor_msgs::msg::PointCloud2>.");
}

PclToRosPointCloud2State::~PclToRosPointCloud2State() {}

std::string
PclToRosPointCloud2State::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    auto output_cloud = common::make_ros_point_cloud2();
    pcl_conversions::fromPCL(*input_cloud, *output_cloud);
    blackboard->set<common::RosPointCloud2Ptr>("output_cloud", output_cloud);

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to convert PCL point cloud to ROS: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::PclToRosPointCloud2State, yasmin::State)
