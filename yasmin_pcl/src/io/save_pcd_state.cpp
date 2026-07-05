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

#include "yasmin_pcl/io/save_pcd_state.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

SavePcdState::SavePcdState() : yasmin::State({"succeeded", "aborted"}) {
  this->file_path_.clear();
  this->storage_mode_ = "binary";
  this->origin_x_ = 0.0F;
  this->origin_y_ = 0.0F;
  this->origin_z_ = 0.0F;
  this->origin_w_ = 0.0F;
  this->orientation_x_ = 0.0F;
  this->orientation_y_ = 0.0F;
  this->orientation_z_ = 0.0F;
  this->orientation_w_ = 1.0F;

  this->set_description(
      "Saves a pcl::PCLPointCloud2 cloud from the blackboard to a PCD file.");
  this->set_outcome_description("succeeded", "The PCD file was written.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or writing failed.");

  this->declare_parameter<std::string>("file_path",
                                       "Path to the PCD file to write.", "");
  this->declare_parameter<std::string>(
      "storage_mode",
      "PCD storage mode. Supported values: ascii, binary, binary_compressed.",
      std::string("binary"));
  this->declare_parameter<float>(
      "origin_x", "Sensor origin x component used for writing.", 0.0F);
  this->declare_parameter<float>(
      "origin_y", "Sensor origin y component used for writing.", 0.0F);
  this->declare_parameter<float>(
      "origin_z", "Sensor origin z component used for writing.", 0.0F);
  this->declare_parameter<float>(
      "origin_w", "Sensor origin w component used for writing.", 0.0F);
  this->declare_parameter<float>("orientation_x",
                                 "Orientation quaternion x component.", 0.0F);
  this->declare_parameter<float>("orientation_y",
                                 "Orientation quaternion y component.", 0.0F);
  this->declare_parameter<float>("orientation_z",
                                 "Orientation quaternion z component.", 0.0F);
  this->declare_parameter<float>("orientation_w",
                                 "Orientation quaternion w component.", 1.0F);

  this->add_input_key("input_cloud",
                      "Input cloud stored as pcl::PCLPointCloud2::Ptr.");
}

void SavePcdState::configure() {
  this->file_path_ = this->get_parameter<std::string>("file_path");
  this->storage_mode_ = this->get_parameter<std::string>("storage_mode");
  this->origin_x_ = this->get_parameter<float>("origin_x");
  this->origin_y_ = this->get_parameter<float>("origin_y");
  this->origin_z_ = this->get_parameter<float>("origin_z");
  this->origin_w_ = this->get_parameter<float>("origin_w");
  this->orientation_x_ = this->get_parameter<float>("orientation_x");
  this->orientation_y_ = this->get_parameter<float>("orientation_y");
  this->orientation_z_ = this->get_parameter<float>("orientation_z");
  this->orientation_w_ = this->get_parameter<float>("orientation_w");
}

std::string SavePcdState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    if (this->is_canceled()) {
      return "aborted";
    }

    if (this->file_path_.empty()) {
      YASMIN_LOG_WARN("Parameter 'file_path' is empty");
      return "aborted";
    }

    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    if (this->is_canceled()) {
      return "aborted";
    }

    const Eigen::Vector4f origin(this->origin_x_, this->origin_y_,
                                 this->origin_z_, this->origin_w_);
    const Eigen::Quaternionf orientation(
        this->orientation_w_, this->orientation_x_, this->orientation_y_,
        this->orientation_z_);

    pcl::PCDWriter writer;
    int result = -1;

    if (this->storage_mode_ == "ascii") {
      result = writer.writeASCII(this->file_path_, *input_cloud, origin,
                                 orientation);
    } else if (this->storage_mode_ == "binary") {
      result = writer.writeBinary(this->file_path_, *input_cloud, origin,
                                  orientation);
    } else if (this->storage_mode_ == "binary_compressed") {
      result = writer.writeBinaryCompressed(this->file_path_, *input_cloud,
                                            origin, orientation);
    } else {
      YASMIN_LOG_WARN("Unsupported PCD storage_mode '%s'",
                      this->storage_mode_.c_str());
      return "aborted";
    }

    if (result < 0) {
      YASMIN_LOG_WARN("Failed to write PCD file '%s'",
                      this->file_path_.c_str());
      return "aborted";
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to save PCD file: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::SavePcdState, yasmin::State)
