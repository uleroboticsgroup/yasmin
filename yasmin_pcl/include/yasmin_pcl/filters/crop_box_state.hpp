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

#ifndef YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_
#define YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::CropBox to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud` and the
 * filtered cloud is written to `output_cloud`. Optional input indices can be
 * provided through `input_indices`. Removed indices are written to
 * `removed_indices` when enabled.
 */
class CropBoxState : public yasmin::State {
public:
  /** @brief Construct a CropBoxState. */
  CropBoxState();
  /** @brief Default destructor. */
  ~CropBoxState() override = default;

  /** @brief Configure the crop box filter from blackboard parameters. */
  void configure() override;
  /** @brief Execute the crop box filter.
   *  @param blackboard The shared blackboard.
   *  @return Outcome string. */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Minimum X bound of the crop box.
  float min_x_;
  /// @brief Minimum Y bound of the crop box.
  float min_y_;
  /// @brief Minimum Z bound of the crop box.
  float min_z_;
  /// @brief Minimum W (homogeneous) bound of the crop box.
  float min_w_;
  /// @brief Maximum X bound of the crop box.
  float max_x_;
  /// @brief Maximum Y bound of the crop box.
  float max_y_;
  /// @brief Maximum Z bound of the crop box.
  float max_z_;
  /// @brief Maximum W (homogeneous) bound of the crop box.
  float max_w_;
  /// @brief Translation offset in X before cropping.
  float translation_x_;
  /// @brief Translation offset in Y before cropping.
  float translation_y_;
  /// @brief Translation offset in Z before cropping.
  float translation_z_;
  /// @brief Rotation roll of the crop box (radians).
  float rotation_roll_;
  /// @brief Rotation pitch of the crop box (radians).
  float rotation_pitch_;
  /// @brief Rotation yaw of the crop box (radians).
  float rotation_yaw_;
  /// @brief Whether to apply the transform offset.
  bool use_transform_;
  /// @brief Transform translation in X.
  float transform_translation_x_;
  /// @brief Transform translation in Y.
  float transform_translation_y_;
  /// @brief Transform translation in Z.
  float transform_translation_z_;
  /// @brief Transform rotation roll (radians).
  float transform_roll_;
  /// @brief Transform rotation pitch (radians).
  float transform_pitch_;
  /// @brief Transform rotation yaw (radians).
  float transform_yaw_;
  /// @brief If true, keep points outside the crop box.
  bool negative_;
  /// @brief Keep the cloud organized after cropping.
  bool keep_organized_;
  /// @brief Value to assign to filtered-out points.
  float user_filter_value_;
  /// @brief Whether to extract and publish removed indices.
  bool extract_removed_indices_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_
