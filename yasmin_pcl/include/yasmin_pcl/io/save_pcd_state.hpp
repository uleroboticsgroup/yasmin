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

#ifndef YASMIN_PCL__IO__SAVE_PCD_STATE_HPP_
#define YASMIN_PCL__IO__SAVE_PCD_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Saves pcl::PCLPointCloud2 to a PCD file.
 *
 * The input cloud is read from the blackboard key `input_cloud`. The target
 * file path and writer options are read from state-local parameters.
 */
class SavePcdState : public yasmin::State {
public:
  /** @brief Construct a SavePcdState. */
  SavePcdState();
  /** @brief Default destructor. */
  ~SavePcdState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /**
   * @brief Save a point cloud to a PCD file.
   * @param blackboard The shared blackboard.
   * @return Outcome string.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Path to the PCD file to save
  std::string file_path_;
  /// @brief Storage mode for the PCD writer
  std::string storage_mode_;
  /// @brief Origin X coordinate
  float origin_x_;
  /// @brief Origin Y coordinate
  float origin_y_;
  /// @brief Origin Z coordinate
  float origin_z_;
  /// @brief Origin W component
  float origin_w_;
  /// @brief Orientation X component
  float orientation_x_;
  /// @brief Orientation Y component
  float orientation_y_;
  /// @brief Orientation Z component
  float orientation_z_;
  /// @brief Orientation W component
  float orientation_w_;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__SAVE_PCD_STATE_HPP_
