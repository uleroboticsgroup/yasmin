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

#ifndef YASMIN_PCL__IO__SAVE_PLY_STATE_HPP_
#define YASMIN_PCL__IO__SAVE_PLY_STATE_HPP_

#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin_pcl::io {

/**
 * @brief Saves pcl::PCLPointCloud2 to a PLY file.
 *
 * The input cloud is read from the blackboard key `input_cloud`. The target
 * file path and writer options are read from state-local parameters.
 */
class SavePlyState : public yasmin::State {
public:
  /** @brief Construct a SavePlyState. */
  SavePlyState();
  /** @brief Default destructor. */
  ~SavePlyState() override = default;

  /** @brief Configure from blackboard parameters. */
  void configure() override;
  /**
   * @brief Save a point cloud to a PLY file.
   * @param blackboard The shared blackboard.
   * @return Outcome string.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Path to the PLY file to save
  std::string file_path_;
  /// @brief Whether to write in binary mode
  bool binary_mode_;
  /// @brief Whether to use camera coordinates
  bool use_camera_;
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

#endif // YASMIN_PCL__IO__SAVE_PLY_STATE_HPP_
