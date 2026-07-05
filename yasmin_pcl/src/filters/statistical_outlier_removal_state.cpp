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

#include "yasmin_pcl/filters/statistical_outlier_removal_state.hpp"

#include <pcl/filters/statistical_outlier_removal.h>

#include <limits>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace yasmin_pcl::filters {

StatisticalOutlierRemovalState::StatisticalOutlierRemovalState()
    : yasmin::State({"succeeded", "aborted"}) {
  mean_k_ = 10;
  stddev_mul_thresh_ = 1.0;
  negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description(
      "Applies pcl::StatisticalOutlierRemoval to a pcl::PCLPointCloud2 "
      "stored in the blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was filtered successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<int>(
      "mean_k", "Number of neighbors used for mean distance estimation.", 10);
  this->declare_parameter<double>(
      "stddev_mul_thresh",
      "Standard deviation multiplier used to classify outliers.", 1.0);
  this->declare_parameter<bool>(
      "negative", "Return the detected outliers instead of the inliers.",
      false);
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

void StatisticalOutlierRemovalState::configure() {
  this->mean_k_ = this->get_parameter<int>("mean_k");
  this->stddev_mul_thresh_ = this->get_parameter<double>("stddev_mul_thresh");
  this->negative_ = this->get_parameter<bool>("negative");
  this->keep_organized_ = this->get_parameter<bool>("keep_organized");
  this->user_filter_value_ = this->get_parameter<float>("user_filter_value");
  this->extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string StatisticalOutlierRemovalState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
  return common::execute_filter<
      pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>>(
      blackboard, "StatisticalOutlierRemoval", this->extract_removed_indices_,
      [this](auto &filter) {
        filter.setMeanK(this->mean_k_);
        filter.setStddevMulThresh(this->stddev_mul_thresh_);
        filter.setNegative(this->negative_);
        filter.setKeepOrganized(this->keep_organized_);
        filter.setUserFilterValue(this->user_filter_value_);
      });
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::StatisticalOutlierRemovalState,
                       yasmin::State)
