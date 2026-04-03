// Copyright (C) 2026 Maik Knof

#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/crop_box_state.hpp"

TEST(CropBoxState, CropsCloudAndStoresRemovedIndices) {
  yasmin_pcl::filters::CropBoxState state;
  state.set_parameter<float>("min_x", -0.5F);
  state.set_parameter<float>("min_y", -0.5F);
  state.set_parameter<float>("min_z", -0.5F);
  state.set_parameter<float>("max_x", 0.5F);
  state.set_parameter<float>("max_y", 0.5F);
  state.set_parameter<float>("max_z", 0.5F);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 1U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].x, 0.0F);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 2U);
}
