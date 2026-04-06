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

#ifndef YASMIN_PCL__TEST__TEST_UTILS_HPP_
#define YASMIN_PCL__TEST__TEST_UTILS_HPP_

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
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

inline pcl::PointCloud<pcl::PointXYZ>
create_xyz_cloud(const std::vector<pcl::PointXYZ> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  cloud.points.assign(points.begin(), points.end());
  return cloud;
}

inline common::PclPointCloud2Ptr
create_pcl_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  auto cloud = common::make_pcl_point_cloud2();
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud = create_xyz_cloud(points);
  pcl::toPCLPointCloud2(xyz_cloud, *cloud);
  return cloud;
}

inline common::RosPointCloud2Ptr
create_ros_cloud_ptr(const std::vector<pcl::PointXYZ> &points) {
  const auto pcl_cloud = create_pcl_cloud_ptr(points);
  auto ros_cloud = common::make_ros_point_cloud2();
  pcl_conversions::fromPCL(*pcl_cloud, *ros_cloud);
  return ros_cloud;
}

inline pcl::PointCloud<pcl::PointXYZ>
to_xyz_cloud(const pcl::PCLPointCloud2 &cloud) {
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(cloud, xyz_cloud);
  return xyz_cloud;
}

inline std::filesystem::path make_temp_path(const std::string &stem,
                                            const std::string &extension) {
  const auto unique_id =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         (stem + "_" + std::to_string(unique_id) + extension);
}

} // namespace yasmin_pcl::test

#endif // YASMIN_PCL__TEST__TEST_UTILS_HPP_
