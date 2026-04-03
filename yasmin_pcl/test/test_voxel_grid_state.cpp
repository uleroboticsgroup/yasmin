// Copyright (C) 2026 Maik Knof

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
