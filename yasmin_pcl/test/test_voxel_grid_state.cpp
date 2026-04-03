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

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/voxel_grid_state.hpp"

TEST(VoxelGridState, DownsamplesCloud) {
  yasmin_pcl::filters::VoxelGridState state;
  state.set_parameter<float>("leaf_size_x", 1.0F);
  state.set_parameter<float>("leaf_size_y", 1.0F);
  state.set_parameter<float>("leaf_size_z", 1.0F);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {0.2F, 0.0F, 0.0F}, {1.5F, 0.0F, 0.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  EXPECT_EQ(xyz_cloud.points.size(), 2U);
}

TEST(VoxelGridState, AppliesFilterFieldBeforeDownsampling) {
  yasmin_pcl::filters::VoxelGridState state;
  state.set_parameter<float>("leaf_size_x", 0.1F);
  state.set_parameter<float>("leaf_size_y", 0.1F);
  state.set_parameter<float>("leaf_size_z", 0.1F);
  state.set_parameter<std::string>("filter_field_name", "z");
  state.set_parameter<double>("filter_limit_min", 0.5);
  state.set_parameter<double>("filter_limit_max", 1.5);
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
}

TEST(VoxelGridState, RestrictsDownsamplingToProvidedInputIndices) {
  yasmin_pcl::filters::VoxelGridState state;
  state.set_parameter<float>("leaf_size_x", 0.1F);
  state.set_parameter<float>("leaf_size_y", 0.1F);
  state.set_parameter<float>("leaf_size_z", 0.1F);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {10.0F, 0.0F, 0.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {0, 1});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  for (const auto &point : xyz_cloud.points) {
    EXPECT_LT(point.x, 2.0F);
  }
}
