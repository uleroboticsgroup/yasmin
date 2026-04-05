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

#ifndef YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_
#define YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

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
  CropBoxState();
  ~CropBoxState() override;

  void configure() override;
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  float min_x_;
  float min_y_;
  float min_z_;
  float min_w_;
  float max_x_;
  float max_y_;
  float max_z_;
  float max_w_;
  float translation_x_;
  float translation_y_;
  float translation_z_;
  float rotation_roll_;
  float rotation_pitch_;
  float rotation_yaw_;
  bool use_transform_;
  float transform_translation_x_;
  float transform_translation_y_;
  float transform_translation_z_;
  float transform_roll_;
  float transform_pitch_;
  float transform_yaw_;
  bool negative_;
  bool keep_organized_;
  float user_filter_value_;
  bool extract_removed_indices_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__CROP_BOX_STATE_HPP_
