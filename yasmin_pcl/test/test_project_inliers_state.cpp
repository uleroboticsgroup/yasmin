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
