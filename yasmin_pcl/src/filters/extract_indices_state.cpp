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

#include "yasmin_pcl/filters/extract_indices_state.hpp"

#include <pcl/filters/extract_indices.h>

#include <exception>
#include <limits>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace {

yasmin_pcl::common::Indices
make_domain_indices(const yasmin_pcl::common::PclPointCloud2Ptr &input_cloud) {
  const auto num_points = static_cast<std::size_t>(input_cloud->width) *
                          static_cast<std::size_t>(input_cloud->height);
  yasmin_pcl::common::Indices indices;
  indices.reserve(num_points);
  for (std::size_t idx = 0; idx < num_points; ++idx) {
    indices.push_back(static_cast<int>(idx));
  }
  return indices;
}

yasmin_pcl::common::Indices
compute_removed_indices(const yasmin_pcl::common::Indices &domain_indices,
                        const yasmin_pcl::common::Indices &output_indices) {
  std::unordered_set<int> selected_indices(output_indices.begin(),
                                           output_indices.end());

  yasmin_pcl::common::Indices removed_indices;
  removed_indices.reserve(domain_indices.size());
  for (const int index : domain_indices) {
    if (selected_indices.count(index) == 0U) {
      removed_indices.push_back(index);
    }
  }

  return removed_indices;
}

} // namespace

namespace yasmin_pcl::filters {

ExtractIndicesState::ExtractIndicesState()
    : yasmin::State({"succeeded", "aborted"}) {
  negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description(
      "Applies pcl::ExtractIndices to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description(
      "succeeded", "The requested indices were extracted successfully.");
  this->set_outcome_description(
      "aborted",
      "The input cloud, indices, or filter configuration were invalid.");

  this->declare_parameter<bool>(
      "negative", "Return points not referenced by input_indices.", false);
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
  this->add_input_key("input_indices",
                      "Indices to extract stored as pcl::Indices.");
  this->add_output_key("output_cloud",
                       "Filtered cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key("output_indices",
                       "Output indices stored as pcl::Indices.");
  this->add_output_key("removed_indices",
                       "Removed point indices stored as pcl::Indices.");
}

ExtractIndicesState::~ExtractIndicesState() {}

void ExtractIndicesState::configure() {
  negative_ = this->get_parameter<bool>("negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string
ExtractIndicesState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    if (!blackboard->contains("input_indices")) {
      YASMIN_LOG_ERROR("Blackboard key 'input_indices' is missing");
      return "aborted";
    }

    pcl::ExtractIndices<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input_cloud);
    filter.setNegative(negative_);
    filter.setKeepOrganized(keep_organized_);
    filter.setUserFilterValue(user_filter_value_);
    common::set_optional_input_indices(filter, blackboard);

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    common::Indices output_indices;
    filter.filter(output_indices);
    blackboard->set<common::Indices>("output_indices", output_indices);

    if (extract_removed_indices_) {
      const auto domain_indices = make_domain_indices(input_cloud);
      blackboard->set<common::Indices>(
          "removed_indices",
          compute_removed_indices(domain_indices, output_indices));
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("ExtractIndices filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::ExtractIndicesState, yasmin::State)
