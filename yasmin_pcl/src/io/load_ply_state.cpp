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

#include "yasmin_pcl/io/load_ply_state.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>
#include <pluginlib/class_list_macros.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::io {

/// Read sensor origin and orientation from the camera element of an ASCII PLY
/// file. PCL 1.12's loadPLYFile does not propagate these values through its
/// output parameters due to a known bug, so we parse them manually here.
/// Returns true when a camera element with viewpoint data was found.
static bool read_ply_camera_ascii(const std::string &file_path,
                                  Eigen::Vector4f &origin,
                                  Eigen::Quaternionf &orientation) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    return false;
  }

  struct Element {
    std::string name;
    std::size_t count{0};
    std::vector<std::string> properties;
  };

  std::vector<Element> elements;
  Element *current_element = nullptr;
  bool in_header = true;
  bool is_ascii = false;
  std::string line;

  while (in_header && std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line == "end_header") {
      in_header = false;
    } else if (line.find("format ascii") == 0) {
      is_ascii = true;
    } else if (line.find("element ") == 0) {
      elements.emplace_back();
      current_element = &elements.back();
      std::istringstream iss(line);
      std::string keyword;
      iss >> keyword >> current_element->name >> current_element->count;
    } else if (line.find("property ") == 0 && current_element) {
      std::istringstream iss(line);
      std::string keyword, type, prop_name;
      iss >> keyword >> type >> prop_name;
      current_element->properties.push_back(prop_name);
    }
  }

  if (in_header || !is_ascii) {
    return false;
  }

  for (const auto &elem : elements) {
    if (elem.name == "camera") {
      if (elem.count == 0) {
        return false;
      }
      if (!std::getline(file, line)) {
        return false;
      }
      std::istringstream iss(line);
      std::vector<float> vals;
      float v = 0.0f;
      while (iss >> v) {
        vals.push_back(v);
      }

      float view_px = 0.0f, view_py = 0.0f, view_pz = 0.0f;
      float x_axisx = 1.0f, x_axisy = 0.0f, x_axisz = 0.0f;
      float y_axisx = 0.0f, y_axisy = 1.0f, y_axisz = 0.0f;
      float z_axisx = 0.0f, z_axisy = 0.0f, z_axisz = 1.0f;

      for (std::size_t k = 0; k < elem.properties.size() && k < vals.size();
           ++k) {
        const auto &p = elem.properties[k];
        if (p == "view_px") {
          view_px = vals[k];
        } else if (p == "view_py") {
          view_py = vals[k];
        } else if (p == "view_pz") {
          view_pz = vals[k];
        } else if (p == "x_axisx") {
          x_axisx = vals[k];
        } else if (p == "x_axisy") {
          x_axisy = vals[k];
        } else if (p == "x_axisz") {
          x_axisz = vals[k];
        } else if (p == "y_axisx") {
          y_axisx = vals[k];
        } else if (p == "y_axisy") {
          y_axisy = vals[k];
        } else if (p == "y_axisz") {
          y_axisz = vals[k];
        } else if (p == "z_axisx") {
          z_axisx = vals[k];
        } else if (p == "z_axisy") {
          z_axisy = vals[k];
        } else if (p == "z_axisz") {
          z_axisz = vals[k];
        }
      }

      origin = Eigen::Vector4f(view_px, view_py, view_pz, 0.0f);
      Eigen::Matrix3f R;
      R << x_axisx, x_axisy, x_axisz, y_axisx, y_axisy, y_axisz, z_axisx,
          z_axisy, z_axisz;
      orientation = Eigen::Quaternionf(R);
      return true;
    }

    // Not the camera element — skip its data rows.
    for (std::size_t i = 0; i < elem.count; ++i) {
      if (!std::getline(file, line)) {
        return false;
      }
    }
  }

  return false;
}

LoadPlyState::LoadPlyState() : yasmin::State({"succeeded", "aborted"}) {
  file_path_.clear();

  this->set_description(
      "Loads a PLY file into pcl::PCLPointCloud2 and stores the cloud and file "
      "metadata in the blackboard.");
  this->set_outcome_description("succeeded", "The PLY file was loaded.");
  this->set_outcome_description("aborted",
                                "The file path was invalid or loading failed.");

  this->declare_parameter<std::string>("file_path",
                                       "Path to the PLY file to load.", "");

  this->add_output_key("output_cloud",
                       "Loaded cloud stored as pcl::PCLPointCloud2::Ptr.");
  this->add_output_key(
      "sensor_origin",
      "Sensor acquisition origin stored as std::array<float, 4>.");
  this->add_output_key("sensor_orientation",
                       "Sensor acquisition orientation quaternion stored as "
                       "std::array<float, 4>.");
}

LoadPlyState::~LoadPlyState() {}

void LoadPlyState::configure() {
  file_path_ = this->get_parameter<std::string>("file_path");
}

std::string LoadPlyState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  if (file_path_.empty()) {
    YASMIN_LOG_ERROR("Parameter 'file_path' is empty");
    return "aborted";
  }

  auto output_cloud = common::make_pcl_point_cloud2();
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

  const int result =
      pcl::io::loadPLYFile(file_path_, *output_cloud, origin, orientation);

  if (result < 0) {
    YASMIN_LOG_ERROR("Failed to load PLY file '%s'", file_path_.c_str());
    return "aborted";
  }

  // PCL 1.12's loadPLYFile does not populate the origin/orientation output
  // parameters for PCLPointCloud2 due to a bug in PLYReader::read. Parse
  // the camera element directly from the file to work around this.
  read_ply_camera_ascii(file_path_, origin, orientation);

  blackboard->set<common::PclPointCloud2Ptr>("output_cloud", output_cloud);
  blackboard->set<common::Vector4fArray>("sensor_origin",
                                         common::to_array(origin));
  blackboard->set<common::Vector4fArray>("sensor_orientation",
                                         common::to_array(orientation));

  return "succeeded";
}

} // namespace yasmin_pcl::io

PLUGINLIB_EXPORT_CLASS(yasmin_pcl::io::LoadPlyState, yasmin::State)
