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

#ifndef YASMIN_PCL__IO__LOAD_PLY_STATE_HPP_
#define YASMIN_PCL__IO__LOAD_PLY_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

namespace yasmin_pcl::io {

/**
 * @brief Loads a PLY file into pcl::PCLPointCloud2.
 *
 * The file path is taken from the state-local parameter `file_path`. The loaded
 * cloud is written to the blackboard key `output_cloud`. Additional metadata is
 * written to `sensor_origin` and `sensor_orientation`.
 */
class LoadPlyState : public yasmin::State {
public:
  LoadPlyState();
  ~LoadPlyState() override;

  void configure() override;
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  std::string file_path_;
};

} // namespace yasmin_pcl::io

#endif // YASMIN_PCL__IO__LOAD_PLY_STATE_HPP_
