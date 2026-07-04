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

#include "yasmin_pcl/filters/project_inliers_state.hpp"

#include <pcl/filters/project_inliers.h>

#include <exception>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/filter_state_utils.hpp"

namespace yasmin_pcl::filters {

ProjectInliersState::ProjectInliersState()
    : yasmin::State({"succeeded", "aborted"}) {
  this->model_type_ = 0;
  this->copy_all_fields_ = true;
  this->copy_all_data_ = false;

  this->set_description(
      "Applies pcl::ProjectInliers to a pcl::PCLPointCloud2 stored in the "
      "blackboard.");
  this->set_outcome_description("succeeded",
                                "The cloud was projected successfully.");
  this->set_outcome_description(
      "aborted",
      "The input cloud, model coefficients, or configuration were invalid.");

  this->declare_parameter<int>(
      "model_type", "Sample consensus model type used for projection.", 0);
  this->declare_parameter<bool>(
      "copy_all_fields",
      "Copy all input fields to the output cloud instead of only XYZ.", true);
  this->declare_parameter<bool>(
      "copy_all_data",
      "Return all points instead of only the projected inliers.", false);

  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_input_key(
      "input_model_coefficients",
      "Model coefficients stored as pcl::ModelCoefficients::Ptr.");
  this->add_input_key(
      "input_indices",
      "Optional subset of point indices stored as pcl::Indices.");
  this->add_output_key("output_cloud",
                       "Projected cloud stored as pcl::PCLPointCloud2::Ptr.");
}

void ProjectInliersState::configure() {
  this->model_type_ = this->get_parameter<int>("model_type");
  this->copy_all_fields_ = this->get_parameter<bool>("copy_all_fields");
  this->copy_all_data_ = this->get_parameter<bool>("copy_all_data");
}

std::string
ProjectInliersState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    if (!blackboard->contains("input_model_coefficients")) {
      YASMIN_LOG_WARN("Blackboard key 'input_model_coefficients' is missing");
      return "aborted";
    }

    const auto coefficients = blackboard->get<common::ModelCoefficientsPtr>(
        "input_model_coefficients");
    if (!coefficients) {
      YASMIN_LOG_WARN("Input model coefficients pointer is null");
      return "aborted";
    }

    pcl::ProjectInliers<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input_cloud);
    filter.setModelType(this->model_type_);
    filter.setModelCoefficients(coefficients);
    filter.setCopyAllFields(this->copy_all_fields_);
    filter.setCopyAllData(this->copy_all_data_);
    common::set_optional_input_indices(filter, blackboard);

    auto output_cloud = common::make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("ProjectInliers filtering failed: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::filters

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::filters::ProjectInliersState, yasmin::State)
