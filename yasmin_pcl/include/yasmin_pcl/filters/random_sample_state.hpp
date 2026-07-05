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

#ifndef YASMIN_PCL__FILTERS__RANDOM_SAMPLE_STATE_HPP_
#define YASMIN_PCL__FILTERS__RANDOM_SAMPLE_STATE_HPP_

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::RandomSample to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud` and the
 * sampled cloud is written to `output_cloud`. Optional input indices can be
 * provided through `input_indices`. The resulting sampled indices are written
 * to `output_indices`.
 */
class RandomSampleState : public yasmin::State {
public:
  /** @brief Construct a RandomSampleState. */
  RandomSampleState();
  /** @brief Default destructor. */
  ~RandomSampleState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /** @brief Execute the random sample filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Number of points to sample.
  int sample_;
  /// @brief Random seed value.
  int seed_;
  /// @brief If true, keep the complement of the sample.
  bool negative_;
  /// @brief Keep the cloud organized after sampling.
  bool keep_organized_;
  /// @brief Value to assign to filtered-out points.
  float user_filter_value_;
  /// @brief Whether to extract and publish removed indices.
  bool extract_removed_indices_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__RANDOM_SAMPLE_STATE_HPP_
