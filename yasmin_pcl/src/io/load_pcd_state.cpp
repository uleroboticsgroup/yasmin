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

#include "yasmin_pcl/io/load_pcd_state.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

LoadPcdState::LoadPcdState() : yasmin::State({"succeeded", "aborted"}) {
  this->file_path_.clear();

  this->set_description(
      "Loads a PCD file into pcl::PCLPointCloud2 and stores the cloud and file "
      "metadata in the blackboard.");
  this->set_outcome_description("succeeded", "The PCD file was loaded.");
  this->set_outcome_description("aborted",
                                "The file path was invalid or loading failed.");

  this->declare_parameter<std::string>("file_path",
                                       "Path to the PCD file to load.", "");

  this->add_output_key("output_cloud",
                       "Loaded cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key(
      "sensor_origin",
      "Sensor acquisition origin stored as std::array<float, 4>.");
  this->add_output_key("sensor_orientation",
                       "Sensor acquisition orientation quaternion stored as "
                       "std::array<float, 4>.");
  this->add_output_key("pcd_version", "Detected PCD version stored as int.");
}

void LoadPcdState::configure() {
  this->file_path_ = this->get_parameter<std::string>("file_path");
}

std::string LoadPcdState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  if (this->file_path_.empty()) {
    YASMIN_LOG_WARN("Parameter 'file_path' is empty");
    return "aborted";
  }

  auto output_cloud = common::make_pcl_point_cloud2();
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  int pcd_version = 0;

  pcl::PCDReader reader;
  const int result = reader.read(this->file_path_, *output_cloud, origin,
                                 orientation, pcd_version);

  if (result < 0) {
    YASMIN_LOG_WARN("Failed to load PCD file '%s'", this->file_path_.c_str());
    return "aborted";
  }

  blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);
  blackboard->set<common::Vector4fArray>("sensor_origin",
                                         common::to_array(origin));
  blackboard->set<common::Vector4fArray>("sensor_orientation",
                                         common::to_array(orientation));
  blackboard->set<int>("pcd_version", pcd_version);

  return "succeeded";
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::LoadPcdState, yasmin::State)
