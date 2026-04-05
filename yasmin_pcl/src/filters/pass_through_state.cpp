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

#include "yasmin_pcl/filters/pass_through_state.hpp"

#include <exception>
#include <limits>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::filters {

PassThroughState::PassThroughState() : yasmin::State({"succeeded", "aborted"}) {
  filter_field_name_ = "z";
  filter_limit_min_ = 0.0;
  filter_limit_max_ = 1.0;
  filter_limit_negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description(
      "Applies pcl::PassThrough to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was filtered successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<std::string>(
      "filter_field_name",
      "Point field used for filtering, for example x, y, z, or intensity.",
      std::string("z"));
  this->declare_parameter<double>("filter_limit_min",
                                  "Lower inclusive filter limit.", 0.0);
  this->declare_parameter<double>("filter_limit_max",
                                  "Upper inclusive filter limit.", 1.0);
  this->declare_parameter<bool>(
      "filter_limit_negative",
      "Return the points outside the interval instead of inside it.", false);
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

PassThroughState::~PassThroughState() {}

void PassThroughState::configure() {
  filter_field_name_ = this->get_parameter<std::string>("filter_field_name");
  filter_limit_min_ = this->get_parameter<double>("filter_limit_min");
  filter_limit_max_ = this->get_parameter<double>("filter_limit_max");
  filter_limit_negative_ = this->get_parameter<bool>("filter_limit_negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string
PassThroughState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::PassThrough<pcl::PCLPointCloud2> filter(extract_removed_indices_);
    filter.setInputCloud(input_cloud);
    filter.setFilterFieldName(filter_field_name_);
    filter.setFilterLimits(filter_limit_min_, filter_limit_max_);
    filter.setNegative(filter_limit_negative_);
    filter.setKeepOrganized(keep_organized_);
    filter.setUserFilterValue(user_filter_value_);

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
    YASMIN_LOG_ERROR("PassThrough filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::PassThroughState, yasmin::State)
