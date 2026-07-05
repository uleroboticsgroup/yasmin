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

#ifndef YASMIN_PCL__FILTERS__PASS_THROUGH_STATE_HPP_
#define YASMIN_PCL__FILTERS__PASS_THROUGH_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::PassThrough to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud` and the
 * filtered cloud is written to `output_cloud`. Optional input indices can be
 * provided through `input_indices`. Removed indices are written to
 * `removed_indices` when enabled.
 */
class PassThroughState : public yasmin::State {
public:
  /** @brief Construct a PassThroughState. */
  PassThroughState();
  /** @brief Default destructor. */
  ~PassThroughState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /** @brief Execute the pass through filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Name of the field to filter on.
  std::string filter_field_name_;
  /// @brief Minimum filter limit value.
  double filter_limit_min_;
  /// @brief Maximum filter limit value.
  double filter_limit_max_;
  /// @brief If true, invert the filter condition.
  bool filter_limit_negative_;
  /// @brief Keep the cloud organized after filtering.
  bool keep_organized_;
  /// @brief Value to assign to filtered-out points.
  float user_filter_value_;
  /// @brief Whether to extract and publish removed indices.
  bool extract_removed_indices_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__PASS_THROUGH_STATE_HPP_
