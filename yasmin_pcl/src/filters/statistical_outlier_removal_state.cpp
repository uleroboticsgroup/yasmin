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

#include "yasmin_pcl/filters/statistical_outlier_removal_state.hpp"

#include <pcl/filters/statistical_outlier_removal.h>

#include <exception>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
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

StatisticalOutlierRemovalState::~StatisticalOutlierRemovalState() {}

void StatisticalOutlierRemovalState::configure() {
  mean_k_ = this->get_parameter<int>("mean_k");
  stddev_mul_thresh_ = this->get_parameter<double>("stddev_mul_thresh");
  negative_ = this->get_parameter<bool>("negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string StatisticalOutlierRemovalState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> filter(
        extract_removed_indices_);
    filter.setInputCloud(input_cloud);
    filter.setMeanK(mean_k_);
    filter.setStddevMulThresh(stddev_mul_thresh_);
    filter.setNegative(negative_);
    filter.setKeepOrganized(keep_organized_);
    filter.setUserFilterValue(user_filter_value_);
    common::set_optional_input_indices(filter, blackboard);

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    if (extract_removed_indices_) {
      common::store_removed_indices(filter, blackboard);
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("StatisticalOutlierRemoval filtering failed: %s",
                     e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::StatisticalOutlierRemovalState,
                       yasmin::State)
