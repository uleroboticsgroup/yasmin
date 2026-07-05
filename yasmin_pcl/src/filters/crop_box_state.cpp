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

#include "yasmin_pcl/filters/crop_box_state.hpp"

#include <pcl/filters/crop_box.h>

#include <Eigen/Geometry>
#include <exception>
#include <limits>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/transform_utils.hpp"

namespace yasmin_pcl::filters {

CropBoxState::CropBoxState() : yasmin::State({"succeeded", "aborted"}) {
  this->min_x_ = -1.0F;
  this->min_y_ = -1.0F;
  this->min_z_ = -1.0F;
  this->min_w_ = 1.0F;
  this->max_x_ = 1.0F;
  this->max_y_ = 1.0F;
  this->max_z_ = 1.0F;
  this->max_w_ = 1.0F;
  this->translation_x_ = 0.0F;
  this->translation_y_ = 0.0F;
  this->translation_z_ = 0.0F;
  this->rotation_roll_ = 0.0F;
  this->rotation_pitch_ = 0.0F;
  this->rotation_yaw_ = 0.0F;
  this->use_transform_ = false;
  this->transform_translation_x_ = 0.0F;
  this->transform_translation_y_ = 0.0F;
  this->transform_translation_z_ = 0.0F;
  this->transform_roll_ = 0.0F;
  this->transform_pitch_ = 0.0F;
  this->transform_yaw_ = 0.0F;
  this->negative_ = false;
  this->keep_organized_ = false;
  this->user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  this->extract_removed_indices_ = false;

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

void CropBoxState::configure() {
  this->min_x_ = this->get_parameter<float>("min_x");
  this->min_y_ = this->get_parameter<float>("min_y");
  this->min_z_ = this->get_parameter<float>("min_z");
  this->min_w_ = this->get_parameter<float>("min_w");
  this->max_x_ = this->get_parameter<float>("max_x");
  this->max_y_ = this->get_parameter<float>("max_y");
  this->max_z_ = this->get_parameter<float>("max_z");
  this->max_w_ = this->get_parameter<float>("max_w");
  this->translation_x_ = this->get_parameter<float>("translation_x");
  this->translation_y_ = this->get_parameter<float>("translation_y");
  this->translation_z_ = this->get_parameter<float>("translation_z");
  this->rotation_roll_ = this->get_parameter<float>("rotation_roll");
  this->rotation_pitch_ = this->get_parameter<float>("rotation_pitch");
  this->rotation_yaw_ = this->get_parameter<float>("rotation_yaw");
  this->use_transform_ = this->get_parameter<bool>("use_transform");
  this->transform_translation_x_ =
      this->get_parameter<float>("transform_translation_x");
  this->transform_translation_y_ =
      this->get_parameter<float>("transform_translation_y");
  this->transform_translation_z_ =
      this->get_parameter<float>("transform_translation_z");
  this->transform_roll_ = this->get_parameter<float>("transform_roll");
  this->transform_pitch_ = this->get_parameter<float>("transform_pitch");
  this->transform_yaw_ = this->get_parameter<float>("transform_yaw");
  this->negative_ = this->get_parameter<bool>("negative");
  this->keep_organized_ = this->get_parameter<bool>("keep_organized");
  this->user_filter_value_ = this->get_parameter<float>("user_filter_value");
  this->extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string CropBoxState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::CropBox<pcl::PCLPointCloud2> filter(this->extract_removed_indices_);
    filter.setInputCloud(input_cloud);
    filter.setMin(Eigen::Vector4f(this->min_x_, this->min_y_, this->min_z_,
                                  this->min_w_));
    filter.setMax(Eigen::Vector4f(this->max_x_, this->max_y_, this->max_z_,
                                  this->max_w_));
    filter.setTranslation(Eigen::Vector3f(
        this->translation_x_, this->translation_y_, this->translation_z_));
    filter.setRotation(Eigen::Vector3f(
        this->rotation_roll_, this->rotation_pitch_, this->rotation_yaw_));
    filter.setNegative(this->negative_);
    filter.setKeepOrganized(this->keep_organized_);
    filter.setUserFilterValue(this->user_filter_value_);

    if (this->use_transform_) {
      filter.setTransform(common::affine_from_translation_rpy(
          this->transform_translation_x_, this->transform_translation_y_,
          this->transform_translation_z_, this->transform_roll_,
          this->transform_pitch_, this->transform_yaw_));
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

    if (this->extract_removed_indices_) {
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
