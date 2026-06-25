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

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Geometry>
#include <array>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "yasmin_pcl/common/pcl_compat.hpp"

namespace yasmin_pcl::common {

/// @brief Shared pointer to a ROS PointCloud2 message.
using RosPointCloud2Ptr = std::shared_ptr<sensor_msgs::msg::PointCloud2>;
/// @brief Shared pointer to a PCL PointCloud2.
using PclPointCloud2Ptr = pcl::PCLPointCloud2::Ptr;
/// @brief Const shared pointer to a PCL PointCloud2.
using PclPointCloud2ConstPtr = pcl::PCLPointCloud2::ConstPtr;
/// @brief PCL point cloud indices.
using Indices = pcl::Indices;
/// @brief PCL model coefficients.
using ModelCoefficients = pcl::ModelCoefficients;
/// @brief Shared pointer to PCL model coefficients.
using ModelCoefficientsPtr = pcl::ModelCoefficients::Ptr;
/// @brief Const shared pointer to PCL model coefficients.
using ModelCoefficientsConstPtr = pcl::ModelCoefficients::ConstPtr;
/// @brief Array of four floats.
using Vector4fArray = std::array<float, 4>;

/** @brief Create an empty ROS PointCloud2 shared pointer.
 *  @return Shared pointer to sensor_msgs::msg::PointCloud2. */
inline RosPointCloud2Ptr make_ros_point_cloud2() {
  return std::make_shared<sensor_msgs::msg::PointCloud2>();
}

/** @brief Create an empty PCL PointCloud2 shared pointer.
 *  @return Shared pointer to pcl::PCLPointCloud2. */
inline PclPointCloud2Ptr make_pcl_point_cloud2() {
  return PclPointCloud2Ptr(new pcl::PCLPointCloud2());
}

/** @brief Convert an Eigen::Vector4f to a std::array<float, 4>.
 *  @param v The Eigen vector to convert.
 *  @return Array containing x, y, z, w components. */
inline Vector4fArray to_array(const Eigen::Vector4f &v) {
  return {v.x(), v.y(), v.z(), v.w()};
}

/** @brief Convert an Eigen::Quaternionf to a std::array<float, 4>.
 *  @param q The quaternion to convert.
 *  @return Array containing x, y, z, w components. */
inline Vector4fArray to_array(const Eigen::Quaternionf &q) {
  return {q.x(), q.y(), q.z(), q.w()};
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__CLOUD_TYPES_HPP_
