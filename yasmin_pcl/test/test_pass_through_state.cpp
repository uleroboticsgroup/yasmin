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

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/pass_through_state.hpp"

TEST(PassThroughState, FiltersCloudAndStoresRemovedIndices) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 0.5);
  state.set_parameter<double>("filter_limit_max", 1.5);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 1.0F);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 2U);
}

TEST(PassThroughState, ReturnsOutsideIntervalWhenNegativeEnabled) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 0.5);
  state.set_parameter<double>("filter_limit_max", 1.5);
  state.set_parameter<bool>("filter_limit_negative", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 0.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].z, 2.0F);
}

TEST(PassThroughState, FiltersOnlyProvidedInputIndices) {
  yasmin_pcl::filters::PassThroughState state;
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 1.5);
  state.set_parameter<double>("filter_limit_max", 3.0);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {1, 2});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 2.0F);
}
