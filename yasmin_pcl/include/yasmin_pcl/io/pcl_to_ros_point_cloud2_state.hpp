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

#ifndef YASMIN_PCL__IO__PCL_TO_ROS_POINT_CLOUD2_STATE_HPP_
#define YASMIN_PCL__IO__PCL_TO_ROS_POINT_CLOUD2_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Converts pcl::PCLPointCloud2 into a ROS PointCloud2 message.
 *
 * The input PCL cloud is read from the blackboard key `input_cloud` and the
 * converted ROS message is written to `output_cloud`.
 */
class PclToRosPointCloud2State : public yasmin::State {
public:
  PclToRosPointCloud2State();
  ~PclToRosPointCloud2State() override;

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__PCL_TO_ROS_POINT_CLOUD2_STATE_HPP_
