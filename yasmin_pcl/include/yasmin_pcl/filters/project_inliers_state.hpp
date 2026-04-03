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

#ifndef YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_
#define YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

namespace yasmin_pcl::filters {

/**
 * @brief Applies pcl::ProjectInliers to pcl::PCLPointCloud2.
 *
 * The input cloud is read from the blackboard key `input_cloud`. Model
 * coefficients are read from `input_model_coefficients`. Optional input indices
 * can be provided through `input_indices`. The projected cloud is written to
 * `output_cloud`.
 */
class ProjectInliersState : public yasmin::State {
public:
  ProjectInliersState();
  ~ProjectInliersState() override;

  void configure() override;
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  int model_type_;
  bool copy_all_fields_;
  bool copy_all_data_;
};

} // namespace yasmin_pcl::filters

#endif // YASMIN_PCL__FILTERS__PROJECT_INLIERS_STATE_HPP_
