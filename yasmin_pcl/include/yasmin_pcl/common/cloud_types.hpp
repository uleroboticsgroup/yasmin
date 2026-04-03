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

#ifndef YASMIN_PCL__COMMON__CLOUD_TYPES_HPP_
#define YASMIN_PCL__COMMON__CLOUD_TYPES_HPP_

#include <array>
#include <memory>

#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace yasmin_pcl::common {

using RosPointCloud2Ptr = std::shared_ptr<sensor_msgs::msg::PointCloud2>;
using PclPointCloud2Ptr = pcl::PCLPointCloud2::Ptr;
using PclPointCloud2ConstPtr = pcl::PCLPointCloud2::ConstPtr;
using Indices = pcl::Indices;
using ModelCoefficients = pcl::ModelCoefficients;
using ModelCoefficientsPtr = pcl::ModelCoefficients::Ptr;
using ModelCoefficientsConstPtr = pcl::ModelCoefficients::ConstPtr;
using Vector4fArray = std::array<float, 4>;

inline RosPointCloud2Ptr make_ros_point_cloud2() {
  return std::make_shared<sensor_msgs::msg::PointCloud2>();
}

inline PclPointCloud2Ptr make_pcl_point_cloud2() {
  return PclPointCloud2Ptr(new pcl::PCLPointCloud2());
}

inline Vector4fArray to_array(const Eigen::Vector4f &value) {
  return {value.x(), value.y(), value.z(), value.w()};
}

inline Vector4fArray to_array(const Eigen::Quaternionf &value) {
  return {value.x(), value.y(), value.z(), value.w()};
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__CLOUD_TYPES_HPP_
