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

#include "yasmin_pcl/filters/voxel_grid_state.hpp"

#include <pcl/filters/voxel_grid.h>

#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace yasmin_pcl::filters {

VoxelGridState::VoxelGridState() : yasmin::State({"succeeded", "aborted"}) {
  leaf_size_x_ = 0.1F;
  leaf_size_y_ = 0.1F;
  leaf_size_z_ = 0.1F;
  downsample_all_data_ = true;
  minimum_points_number_per_voxel_ = 0;
  save_leaf_layout_ = false;
  filter_field_name_.clear();
  filter_limit_min_ = 0.0;
  filter_limit_max_ = 0.0;
  filter_limit_negative_ = false;

  this->set_description(
      "Applies pcl::VoxelGrid to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was downsampled successfully.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or filtering failed.");

  this->declare_parameter<float>("leaf_size_x", "Voxel leaf size along x.",
                                 0.1F);
  this->declare_parameter<float>("leaf_size_y", "Voxel leaf size along y.",
                                 0.1F);
  this->declare_parameter<float>("leaf_size_z", "Voxel leaf size along z.",
                                 0.1F);
  this->declare_parameter<bool>(
      "downsample_all_data",
      "Downsample all fields instead of only the XYZ fields.", true);
  this->declare_parameter<int>(
      "minimum_points_number_per_voxel",
      "Minimum number of points required for a voxel to be kept.", 0);
  this->declare_parameter<bool>(
      "save_leaf_layout",
      "Store leaf layout information for later neighborhood queries.", false);
  this->declare_parameter<std::string>("filter_field_name",
                                       "Optional field name used together with "
                                       "filter limits before downsampling.",
                                       std::string(""));
  this->declare_parameter<double>(
      "filter_limit_min", "Minimum field value used for pre-filtering.", 0.0);
  this->declare_parameter<double>(
      "filter_limit_max", "Maximum field value used for pre-filtering.", 0.0);
  this->declare_parameter<bool>(
      "filter_limit_negative",
      "Return points outside the specified field interval before downsampling.",
      false);

  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_input_key(
      "input_indices",
      "Optional subset of point indices stored as pcl::Indices.");
  this->add_output_key("output_cloud",
                       "Filtered cloud stored as pcl::PCLPointCloud2::Ptr.");
}

VoxelGridState::~VoxelGridState() {}

void VoxelGridState::configure() {
  leaf_size_x_ = this->get_parameter<float>("leaf_size_x");
  leaf_size_y_ = this->get_parameter<float>("leaf_size_y");
  leaf_size_z_ = this->get_parameter<float>("leaf_size_z");
  downsample_all_data_ = this->get_parameter<bool>("downsample_all_data");
  minimum_points_number_per_voxel_ =
      this->get_parameter<int>("minimum_points_number_per_voxel");
  save_leaf_layout_ = this->get_parameter<bool>("save_leaf_layout");
  filter_field_name_ = this->get_parameter<std::string>("filter_field_name");
  filter_limit_min_ = this->get_parameter<double>("filter_limit_min");
  filter_limit_max_ = this->get_parameter<double>("filter_limit_max");
  filter_limit_negative_ = this->get_parameter<bool>("filter_limit_negative");
}

std::string VoxelGridState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input_cloud);
    filter.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    filter.setDownsampleAllData(downsample_all_data_);
    filter.setMinimumPointsNumberPerVoxel(
        static_cast<unsigned int>(minimum_points_number_per_voxel_));
    filter.setSaveLeafLayout(save_leaf_layout_);

    if (!filter_field_name_.empty()) {
      filter.setFilterFieldName(filter_field_name_);
      filter.setFilterLimits(filter_limit_min_, filter_limit_max_);
      filter.setFilterLimitsNegative(filter_limit_negative_);
    }

    common::set_optional_input_indices(filter, blackboard);

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("VoxelGrid filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::VoxelGridState, yasmin::State)
