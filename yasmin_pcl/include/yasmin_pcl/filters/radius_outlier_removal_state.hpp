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

#ifndef YASMIN_PCL__FILTERS__RADIUS_OUTLIER_REMOVAL_STATE_HPP_
#define YASMIN_PCL__FILTERS__RADIUS_OUTLIER_REMOVAL_STATE_HPP_

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::RadiusOutlierRemoval to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud` and the
 * filtered cloud is written to `output_cloud`. Optional input indices can be
 * provided through `input_indices`. Removed indices are written to
 * `removed_indices` when enabled.
 */
class RadiusOutlierRemovalState : public yasmin::State {
public:
  /** @brief Construct a RadiusOutlierRemovalState. */
  RadiusOutlierRemovalState();
  /** @brief Default destructor. */
  ~RadiusOutlierRemovalState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /** @brief Execute the radius outlier removal filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Radius for neighbor search.
  double radius_search_;
  /// @brief Minimum number of neighbors required.
  int min_neighbors_in_radius_;
  /// @brief If true, keep outliers instead of inliers.
  bool negative_;
  /// @brief Keep the cloud organized after removal.
  bool keep_organized_;
  /// @brief Value to assign to filtered-out points.
  float user_filter_value_;
  /// @brief Whether to extract and publish removed indices.
  bool extract_removed_indices_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__RADIUS_OUTLIER_REMOVAL_STATE_HPP_
