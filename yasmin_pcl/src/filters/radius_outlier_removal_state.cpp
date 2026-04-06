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

#include "yasmin_pcl/filters/radius_outlier_removal_state.hpp"

#include <pcl/filters/radius_outlier_removal.h>

#include <exception>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace yasmin_pcl::filters {

RadiusOutlierRemovalState::RadiusOutlierRemovalState()
    : yasmin::State({"succeeded", "aborted"}) {
  radius_search_ = 0.1;
  min_neighbors_in_radius_ = 1;
  negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description(
      "Applies pcl::RadiusOutlierRemoval to a pcl::PCLPointCloud2 stored in "
      "the blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was filtered successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<double>(
      "radius_search",
      "Neighborhood radius used to count neighbors for each point.", 0.1);
  this->declare_parameter<int>(
      "min_neighbors_in_radius",
      "Minimum number of neighbors required to keep a point.", 1);
  this->declare_parameter<bool>(
      "negative", "Return removed outliers instead of inliers.", false);
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

RadiusOutlierRemovalState::~RadiusOutlierRemovalState() {}

void RadiusOutlierRemovalState::configure() {
  radius_search_ = this->get_parameter<double>("radius_search");
  min_neighbors_in_radius_ =
      this->get_parameter<int>("min_neighbors_in_radius");
  negative_ = this->get_parameter<bool>("negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string
RadiusOutlierRemovalState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter(
        extract_removed_indices_);
    filter.setInputCloud(input_cloud);
    filter.setRadiusSearch(radius_search_);
    filter.setMinNeighborsInRadius(min_neighbors_in_radius_);
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
    YASMIN_LOG_ERROR("RadiusOutlierRemoval filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::RadiusOutlierRemovalState,
                       yasmin::State)
