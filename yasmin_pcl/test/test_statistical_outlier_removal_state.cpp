// Copyright (C) 2026 Maik Knof

#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/statistical_outlier_removal_state.hpp"

TEST(StatisticalOutlierRemovalState, RemovesOutlier) {
  yasmin_pcl::filters::StatisticalOutlierRemovalState state;
  state.set_parameter<int>("mean_k", 3);
  state.set_parameter<double>("stddev_mul_thresh", 0.5);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud",
      yasmin_pcl::test::create_pcl_cloud_ptr({{0.00F, 0.00F, 0.00F},
                                              {0.05F, 0.00F, 0.00F},
                                              {0.00F, 0.05F, 0.00F},
                                              {0.05F, 0.05F, 0.00F},
                                              {0.02F, 0.02F, 0.00F},
                                              {10.0F, 10.0F, 10.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  EXPECT_EQ(xyz_cloud.points.size(), 5U);
  for (const auto &point : xyz_cloud.points) {
    EXPECT_LT(point.x, 1.0F);
  }

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 1U);
}
