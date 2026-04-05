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

#ifndef YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_
#define YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

namespace yasmin_pcl::io {

/**
 * @brief Converts a ROS PointCloud2 message into pcl::PCLPointCloud2.
 *
 * The input ROS message is read from the blackboard key `input_cloud` and the
 * converted PCL cloud is written to `output_cloud`.
 */
class RosToPclPointCloud2State : public yasmin::State {
public:
  RosToPclPointCloud2State();
  ~RosToPclPointCloud2State() override;

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_
