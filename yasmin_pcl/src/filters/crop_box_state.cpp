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

#include "yasmin_pcl/filters/crop_box_state.hpp"

#include <exception>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/transform_utils.hpp"

namespace yasmin_pcl::filters {

CropBoxState::CropBoxState() : yasmin::State({"succeeded", "aborted"}) {
  min_x_ = -1.0F;
  min_y_ = -1.0F;
  min_z_ = -1.0F;
  min_w_ = 1.0F;
  max_x_ = 1.0F;
  max_y_ = 1.0F;
  max_z_ = 1.0F;
  max_w_ = 1.0F;
  translation_x_ = 0.0F;
  translation_y_ = 0.0F;
  translation_z_ = 0.0F;
  rotation_roll_ = 0.0F;
  rotation_pitch_ = 0.0F;
  rotation_yaw_ = 0.0F;
  use_transform_ = false;
  transform_translation_x_ = 0.0F;
  transform_translation_y_ = 0.0F;
  transform_translation_z_ = 0.0F;
  transform_roll_ = 0.0F;
  transform_pitch_ = 0.0F;
  transform_yaw_ = 0.0F;
  negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description("Applies pcl::CropBox to a pcl::PCLPointCloud2 stored "
                        "in the blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was cropped successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<float>("min_x", "Crop box minimum x bound.", -1.0F);
  this->declare_parameter<float>("min_y", "Crop box minimum y bound.", -1.0F);
  this->declare_parameter<float>("min_z", "Crop box minimum z bound.", -1.0F);
  this->declare_parameter<float>("min_w", "Crop box minimum homogeneous w.",
                                 1.0F);
  this->declare_parameter<float>("max_x", "Crop box maximum x bound.", 1.0F);
  this->declare_parameter<float>("max_y", "Crop box maximum y bound.", 1.0F);
  this->declare_parameter<float>("max_z", "Crop box maximum z bound.", 1.0F);
  this->declare_parameter<float>("max_w", "Crop box maximum homogeneous w.",
                                 1.0F);
  this->declare_parameter<float>("translation_x",
                                 "Crop box translation along x.", 0.0F);
  this->declare_parameter<float>("translation_y",
                                 "Crop box translation along y.", 0.0F);
  this->declare_parameter<float>("translation_z",
                                 "Crop box translation along z.", 0.0F);
  this->declare_parameter<float>("rotation_roll",
                                 "Crop box roll rotation in radians.", 0.0F);
  this->declare_parameter<float>("rotation_pitch",
                                 "Crop box pitch rotation in radians.", 0.0F);
  this->declare_parameter<float>("rotation_yaw",
                                 "Crop box yaw rotation in radians.", 0.0F);
  this->declare_parameter<bool>(
      "use_transform",
      "Apply an additional affine transform to the cloud before cropping.",
      false);
  this->declare_parameter<float>(
      "transform_translation_x",
      "Additional transform translation x component.", 0.0F);
  this->declare_parameter<float>(
      "transform_translation_y",
      "Additional transform translation y component.", 0.0F);
  this->declare_parameter<float>(
      "transform_translation_z",
      "Additional transform translation z component.", 0.0F);
  this->declare_parameter<float>("transform_roll",
                                 "Additional transform roll in radians.", 0.0F);
  this->declare_parameter<float>(
      "transform_pitch", "Additional transform pitch in radians.", 0.0F);
  this->declare_parameter<float>("transform_yaw",
                                 "Additional transform yaw in radians.", 0.0F);
  this->declare_parameter<bool>("negative",
                                "Return points outside the crop box.", false);
  this->declare_parameter<bool>(
      "keep_organized",
      "Keep the organized cloud structure by replacing filtered points.",
      false);
  this->declare_parameter<float>(
      "user_filter_value",
      "Replacement value used when keep_organized is enabled.",
      std::numeric_limits<float>::quiet_NaN());
  this->declare_parameter<bool>("extract_removed_indices",
                                "Store removed point indices in the blackboard "
                                "output key removed_indices.",
                                false);

  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_input_key(
      "input_indices",
      "Optional subset of point indices stored as pcl::Indices.");
  this->add_output_key("output_cloud",
                       "Filtered cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key("removed_indices",
                       "Removed point indices stored as pcl::Indices.");
}

CropBoxState::~CropBoxState() {}

void CropBoxState::configure() {
  min_x_ = this->get_parameter<float>("min_x");
  min_y_ = this->get_parameter<float>("min_y");
  min_z_ = this->get_parameter<float>("min_z");
  min_w_ = this->get_parameter<float>("min_w");
  max_x_ = this->get_parameter<float>("max_x");
  max_y_ = this->get_parameter<float>("max_y");
  max_z_ = this->get_parameter<float>("max_z");
  max_w_ = this->get_parameter<float>("max_w");
  translation_x_ = this->get_parameter<float>("translation_x");
  translation_y_ = this->get_parameter<float>("translation_y");
  translation_z_ = this->get_parameter<float>("translation_z");
  rotation_roll_ = this->get_parameter<float>("rotation_roll");
  rotation_pitch_ = this->get_parameter<float>("rotation_pitch");
  rotation_yaw_ = this->get_parameter<float>("rotation_yaw");
  use_transform_ = this->get_parameter<bool>("use_transform");
  transform_translation_x_ =
      this->get_parameter<float>("transform_translation_x");
  transform_translation_y_ =
      this->get_parameter<float>("transform_translation_y");
  transform_translation_z_ =
      this->get_parameter<float>("transform_translation_z");
  transform_roll_ = this->get_parameter<float>("transform_roll");
  transform_pitch_ = this->get_parameter<float>("transform_pitch");
  transform_yaw_ = this->get_parameter<float>("transform_yaw");
  negative_ = this->get_parameter<bool>("negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string CropBoxState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::CropBox<pcl::PCLPointCloud2> filter(extract_removed_indices_);
    filter.setInputCloud(input_cloud);
    filter.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, min_w_));
    filter.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, max_w_));
    filter.setTranslation(
        Eigen::Vector3f(translation_x_, translation_y_, translation_z_));
    filter.setRotation(
        Eigen::Vector3f(rotation_roll_, rotation_pitch_, rotation_yaw_));
    filter.setNegative(negative_);
    filter.setKeepOrganized(keep_organized_);
    filter.setUserFilterValue(user_filter_value_);

    if (use_transform_) {
      filter.setTransform(common::affine_from_translation_rpy(
          transform_translation_x_, transform_translation_y_,
          transform_translation_z_, transform_roll_, transform_pitch_,
          transform_yaw_));
    }

    if (blackboard->contains("input_indices")) {
      const auto input_indices =
          blackboard->get<common::Indices>("input_indices");
      pcl::IndicesPtr input_indices_ptr(new pcl::Indices(input_indices));
      filter.setIndices(input_indices_ptr);
    }

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    if (extract_removed_indices_) {
      const auto removed_indices_ptr = filter.getRemovedIndices();
      if (removed_indices_ptr) {
        blackboard->set<common::Indices>("removed_indices",
                                         *removed_indices_ptr);
      } else {
        blackboard->set<common::Indices>("removed_indices", common::Indices{});
      }
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("CropBox filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::CropBoxState, yasmin::State)
