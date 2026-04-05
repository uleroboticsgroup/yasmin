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

#include "yasmin_pcl/filters/random_sample_state.hpp"

#include <exception>
#include <limits>
#include <unordered_set>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
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

RandomSampleState::RandomSampleState()
    : yasmin::State({"succeeded", "aborted"}) {
  sample_ = 1;
  seed_ = 0;
  negative_ = false;
  keep_organized_ = false;
  user_filter_value_ = std::numeric_limits<float>::quiet_NaN();
  extract_removed_indices_ = false;

  this->set_description(
      "Applies pcl::RandomSample to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was sampled successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<int>(
      "sample", "Number of samples to draw from the input cloud.", 1);
  this->declare_parameter<int>(
      "seed", "Seed used by the internal random generator.", 0);
  this->declare_parameter<bool>("negative",
                                "Return points that were not sampled.", false);
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
                       "Sampled cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key("output_indices",
                       "Sampled indices stored as pcl::Indices.");
  this->add_output_key("removed_indices",
                       "Removed point indices stored as pcl::Indices.");
}

RandomSampleState::~RandomSampleState() {}

void RandomSampleState::configure() {
  sample_ = this->get_parameter<int>("sample");
  seed_ = this->get_parameter<int>("seed");
  negative_ = this->get_parameter<bool>("negative");
  keep_organized_ = this->get_parameter<bool>("keep_organized");
  user_filter_value_ = this->get_parameter<float>("user_filter_value");
  extract_removed_indices_ =
      this->get_parameter<bool>("extract_removed_indices");
}

std::string
RandomSampleState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::RandomSample<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input_cloud);
    filter.setSample(static_cast<unsigned int>(sample_));
    filter.setSeed(static_cast<unsigned int>(seed_));
    filter.setNegative(negative_);
    filter.setKeepOrganized(keep_organized_);
    filter.setUserFilterValue(user_filter_value_);
    common::set_optional_input_indices(filter, blackboard);

    common::Indices output_indices;
    filter.filter(output_indices);
    blackboard->set<common::Indices>("output_indices", output_indices);

    pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
    extractor.setInputCloud(input_cloud);
    extractor.setIndices(pcl::IndicesPtr(new pcl::Indices(output_indices)));
    extractor.setNegative(false);
    extractor.setKeepOrganized(keep_organized_);
    extractor.setUserFilterValue(user_filter_value_);

    auto output_cloud = common::make_pcl_point_cloud2();
    extractor.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    if (extract_removed_indices_) {
      const auto domain_indices = make_domain_indices(input_cloud);
      blackboard->set<common::Indices>(
          "removed_indices",
          compute_removed_indices(domain_indices, output_indices));
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("RandomSample filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::RandomSampleState, yasmin::State)
