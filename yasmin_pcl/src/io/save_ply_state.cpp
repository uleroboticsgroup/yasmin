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

#include "yasmin_pcl/io/save_ply_state.hpp"

#include <pcl/io/ply_io.h>

#include <Eigen/Geometry>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

SavePlyState::SavePlyState() : yasmin::State({"succeeded", "aborted"}) {
  file_path_.clear();
  binary_mode_ = false;
  use_camera_ = true;
  origin_x_ = 0.0F;
  origin_y_ = 0.0F;
  origin_z_ = 0.0F;
  origin_w_ = 0.0F;
  orientation_x_ = 0.0F;
  orientation_y_ = 0.0F;
  orientation_z_ = 0.0F;
  orientation_w_ = 1.0F;

  this->set_description(
      "Saves a pcl::PCLPointCloud2 cloud from the blackboard to a PLY file.");
  this->set_outcome_description("succeeded", "The PLY file was written.");
  this->set_outcome_description(
      "aborted", "The input cloud was invalid or writing failed.");

  this->declare_parameter<std::string>("file_path",
                                       "Path to the PLY file to write.", "");
  this->declare_parameter<bool>("binary_mode",
                                "Write the PLY file in binary format.", false);
  this->declare_parameter<bool>(
      "use_camera",
      "Write the PLY file with camera element metadata when supported.", true);
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

SavePlyState::~SavePlyState() {}

void SavePlyState::configure() {
  file_path_ = this->get_parameter<std::string>("file_path");
  binary_mode_ = this->get_parameter<bool>("binary_mode");
  use_camera_ = this->get_parameter<bool>("use_camera");
  origin_x_ = this->get_parameter<float>("origin_x");
  origin_y_ = this->get_parameter<float>("origin_y");
  origin_z_ = this->get_parameter<float>("origin_z");
  origin_w_ = this->get_parameter<float>("origin_w");
  orientation_x_ = this->get_parameter<float>("orientation_x");
  orientation_y_ = this->get_parameter<float>("orientation_y");
  orientation_z_ = this->get_parameter<float>("orientation_z");
  orientation_w_ = this->get_parameter<float>("orientation_w");
}

std::string SavePlyState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    if (file_path_.empty()) {
      YASMIN_LOG_ERROR("Parameter 'file_path' is empty");
      return "aborted";
    }

    const auto input_cloud =
        blackboard->get<common::PclPointCloud2Ptr>("input_cloud");

    if (!input_cloud) {
      YASMIN_LOG_ERROR("Input PCL point cloud pointer is null");
      return "aborted";
    }

    const Eigen::Vector4f origin(origin_x_, origin_y_, origin_z_, origin_w_);
    const Eigen::Quaternionf orientation(orientation_w_, orientation_x_,
                                         orientation_y_, orientation_z_);

    const int result =
        pcl::io::savePLYFile(file_path_, *input_cloud, origin, orientation,
                             binary_mode_, use_camera_);

    if (result < 0) {
      YASMIN_LOG_ERROR("Failed to write PLY file '%s'", file_path_.c_str());
      return "aborted";
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Failed to save PLY file: %s", e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::SavePlyState, yasmin::State)
