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

#ifndef YASMIN_PCL__TEST__TEST_UTILS_HPP_
#define YASMIN_PCL__TEST__TEST_UTILS_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::test {

/** @brief Create a PCL XYZ point cloud from a vector of points.
 *  @param points The input points.
 *  @return The created point cloud. */
inline pcl::PointCloud<pcl::PointXYZ>
create_xyz_cloud(const std::vector<pcl::PointXYZ> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  cloud.points.assign(points.begin(), points.end());
  return cloud;
}

/** @brief Create a PCL PointCloud2 shared pointer from points.
 *  @param points The input points.
 *  @return Shared pointer to a PCL PointCloud2. */
inline common::PclPointCloud2Ptr
create_pcl_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  auto cloud = common::make_pcl_point_cloud2();
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud = create_xyz_cloud(points);
  pcl::toPCLPointCloud2(xyz_cloud, *cloud);
  return cloud;
}

/** @brief Create a ROS PointCloud2 shared pointer from points.
 *  @param points The input points.
 *  @return Shared pointer to a ROS PointCloud2. */
inline common::RosPointCloud2Ptr
create_ros_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  const auto pcl_cloud = create_pcl_cloud_ptr(points);
  auto ros_cloud = common::make_ros_point_cloud2();
  pcl_conversions::fromPCL(*pcl_cloud, *ros_cloud);
  return ros_cloud;
}

/** @brief Convert a PCLPointCloud2 to a PCL XYZ point cloud.
 *  @param cloud The input PCLPointCloud2.
 *  @return The converted XYZ point cloud. */
inline pcl::PointCloud<pcl::PointXYZ>
to_xyz_cloud(const pcl::PCLPointCloud2 &cloud) {
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(cloud, xyz_cloud);
  return xyz_cloud;
}

/** @brief Create a unique temporary file path.
 *  @param stem The filename stem (prefix).
 *  @param suffix The file extension (suffix).
 *  @return A unique temporary file path. */
inline std::filesystem::path make_temp_path(const std::string &stem,
                                            const std::string &suffix) {
  const auto unique_id =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         (stem + "_" + std::to_string(unique_id) + suffix);
}

} // namespace yasmin_pcl::test

#endif // YASMIN_PCL__TEST__TEST_UTILS_HPP_
