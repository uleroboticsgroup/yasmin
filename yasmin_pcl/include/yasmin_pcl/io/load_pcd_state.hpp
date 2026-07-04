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

#ifndef YASMIN_PCL__IO__LOAD_PCD_STATE_HPP_
#define YASMIN_PCL__IO__LOAD_PCD_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Loads a PCD file into pcl::PCLPointCloud2.
 *
 * The file path is taken from the state-local parameter `file_path`. The loaded
 * cloud is written to the blackboard key `output_cloud`. Additional metadata is
 * written to `sensor_origin`, `sensor_orientation`, and `pcd_version`.
 */
class LoadPcdState : public yasmin::State {
public:
  /** @brief Construct a LoadPcdState. */
  LoadPcdState();
  /** @brief Default destructor. */
  ~LoadPcdState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /**
   * @brief Load a PCD file and store the point cloud.
   * @param blackboard The shared blackboard.
   * @return Outcome string.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Path to the PCD file to load
  std::string file_path_;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__LOAD_PCD_STATE_HPP_
