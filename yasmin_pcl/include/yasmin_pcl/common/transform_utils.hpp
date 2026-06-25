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

#ifndef YASMIN_PCL__COMMON__TRANSFORM_UTILS_HPP_
#define YASMIN_PCL__COMMON__TRANSFORM_UTILS_HPP_

#include <Eigen/Geometry>
#include <cmath>

namespace yasmin_pcl::common {

/** @brief Compute a 3x3 rotation matrix from roll, pitch, yaw angles.
 *  @param roll Rotation around X axis (radians).
 *  @param pitch Rotation around Y axis (radians).
 *  @param yaw Rotation around Z axis (radians).
 *  @return 3x3 rotation matrix. */
inline Eigen::Matrix3f
rotation_matrix_from_rpy(const float roll, const float pitch, const float yaw) {
  const Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
  return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

/** @brief Create an affine transform from translation and RPY angles.
 *  @param translation_x Translation along X axis.
 *  @param translation_y Translation along Y axis.
 *  @param translation_z Translation along Z axis.
 *  @param roll Rotation around X axis (radians).
 *  @param pitch Rotation around Y axis (radians).
 *  @param yaw Rotation around Z axis (radians).
 *  @return Affine transform. */
inline Eigen::Affine3f affine_from_translation_rpy(const float translation_x,
                                                   const float translation_y,
                                                   const float translation_z,
                                                   const float roll,
                                                   const float pitch,
                                                   const float yaw) {
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() =
      Eigen::Vector3f(translation_x, translation_y, translation_z);
  transform.linear() = rotation_matrix_from_rpy(roll, pitch, yaw);
  return transform;
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__TRANSFORM_UTILS_HPP_
