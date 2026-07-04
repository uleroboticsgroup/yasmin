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

#ifndef YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_
#define YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::ProjectInliers to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud`. Model
 * coefficients are read from `input_model_coefficients`. Optional input indices
 * can be provided through `input_indices`. The projected cloud is written to
 * `output_cloud`.
 */
class ProjectInliersState : public yasmin::State {
public:
  /** @brief Construct a ProjectInliersState. */
  ProjectInliersState();
  /** @brief Default destructor. */
  ~ProjectInliersState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /** @brief Execute the project inliers filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief PCL model type identifier (e.g., plane, sphere).
  int model_type_;
  /// @brief Copy all fields from input to output.
  bool copy_all_fields_;
  /// @brief Copy all data from input to output.
  bool copy_all_data_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_
