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

#include "yasmin_pcl/io/load_pcd_state.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

LoadPcdState::LoadPcdState() : yasmin::State({"succeeded", "aborted"}) {
  file_path_.clear();

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

LoadPcdState::~LoadPcdState() {}

void LoadPcdState::configure() {
  file_path_ = this->get_parameter<std::string>("file_path");
}

std::string LoadPcdState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  if (file_path_.empty()) {
    YASMIN_LOG_WARN("Parameter 'file_path' is empty");
    return "aborted";
  }

  auto output_cloud = common::make_pcl_point_cloud2();
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  int pcd_version = 0;

  pcl::PCDReader reader;
  const int result =
      reader.read(file_path_, *output_cloud, origin, orientation, pcd_version);

  if (result < 0) {
    YASMIN_LOG_WARN("Failed to load PCD file '%s'", file_path_.c_str());
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
