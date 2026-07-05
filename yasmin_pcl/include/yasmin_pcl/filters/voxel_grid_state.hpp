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

#ifndef YASMIN_PCL__FILTERS__VOXEL_GRID_STATE_HPP_
#define YASMIN_PCL__FILTERS__VOXEL_GRID_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::VoxelGrid to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud` and the
 * filtered cloud is written to `output_cloud`. Optional input indices can be
 * provided through `input_indices`.
 */
class VoxelGridState : public yasmin::State {
public:
  /** @brief Construct a VoxelGridState. */
  VoxelGridState();
  /** @brief Default destructor. */
  ~VoxelGridState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /** @brief Execute the voxel grid filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Voxel leaf size in X.
  float leaf_size_x_;
  /// @brief Voxel leaf size in Y.
  float leaf_size_y_;
  /// @brief Voxel leaf size in Z.
  float leaf_size_z_;
  /// @brief Downsample all data fields (vs. only XYZ).
  bool downsample_all_data_;
  /// @brief Minimum number of points per voxel to keep.
  int minimum_points_number_per_voxel_;
  /// @brief Save the leaf layout information.
  bool save_leaf_layout_;
  /// @brief Name of the field to filter on.
  std::string filter_field_name_;
  /// @brief Minimum filter limit value.
  double filter_limit_min_;
  /// @brief Maximum filter limit value.
  double filter_limit_max_;
  /// @brief If true, invert the filter condition.
  bool filter_limit_negative_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__VOXEL_GRID_STATE_HPP_
