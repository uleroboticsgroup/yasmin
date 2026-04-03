// Copyright (C) 2026 Maik Knof

#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/extract_indices_state.hpp"

TEST(ExtractIndicesState, ExtractsRequestedIndices) {
  yasmin_pcl::filters::ExtractIndicesState state;
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr(
          {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}}));
  blackboard->set<yasmin_pcl::common::Indices>("input_indices", {0, 2});

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].x, 0.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[1].x, 2.0F);

  const auto output_indices =
      blackboard->get<yasmin_pcl::common::Indices>("output_indices");
  EXPECT_EQ(output_indices.size(), 2U);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 1U);
}
