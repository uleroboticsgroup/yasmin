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

#ifndef YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_
#define YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Converts a ROS PointCloud2 message into pcl::PCLPointCloud2.
 *
 * The input ROS message is read from the blackboard key `input_cloud` and the
 * converted PCL cloud is written to `output_cloud`.
 */
class RosToPclPointCloud2State : public yasmin::State {
public:
  /** @brief Construct a RosToPclPointCloud2State. */
  RosToPclPointCloud2State();
  /** @brief Default destructor. */
  ~RosToPclPointCloud2State() override = default;

  /**
   * @brief Convert a ROS PointCloud2 message to a PCL point cloud.
   * @param blackboard The shared blackboard.
   * @return Outcome string.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__ROS_TO_PCL_POINT_CLOUD2_STATE_HPP_
