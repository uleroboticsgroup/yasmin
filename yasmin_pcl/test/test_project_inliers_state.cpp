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

#include <gtest/gtest.h>

#include <pcl/sample_consensus/model_types.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/project_inliers_state.hpp"

TEST(ProjectInliersState, ProjectsPointsOntoPlane) {
  yasmin_pcl::filters::ProjectInliersState state;
  state.set_parameter<int>("model_type", pcl::SACMODEL_PLANE);
  state.set_parameter<bool>("copy_all_fields", true);
  state.set_parameter<bool>("copy_all_data", false);
  state.configure();

  auto coefficients = yasmin_pcl::common::ModelCoefficientsPtr(
      new yasmin_pcl::common::ModelCoefficients());
  coefficients->values = {0.0F, 0.0F, 1.0F, 0.0F};

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{1.0F, 2.0F, 1.0F}, {2.0F, 3.0F, 2.0F}}));
  blackboard->set<yasmin_pcl::common::ModelCoefficientsPtr>(
      "input_model_coefficients", coefficients);

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 0.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].z, 0.0F);
}

TEST(ProjectInliersState, AbortsWhenModelCoefficientsAreMissing) {
  yasmin_pcl::filters::ProjectInliersState state;
  state.set_parameter<int>("model_type", pcl::SACMODEL_PLANE);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{1.0F, 2.0F, 1.0F}, {2.0F, 3.0F, 2.0F}}));

  EXPECT_EQ(state(blackboard), "aborted");
}
