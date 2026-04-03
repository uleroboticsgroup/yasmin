// Copyright (C) 2026 Maik Knof

#include <gtest/gtest.h>

#include <algorithm>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/filters/random_sample_state.hpp"

TEST(RandomSampleState, SamplesRequestedNumberOfPoints) {
  yasmin_pcl::filters::RandomSampleState state;
  state.set_parameter<int>("sample", 2);
  state.set_parameter<int>("seed", 7);
  state.set_parameter<bool>("extract_removed_indices", true);
  state.configure();

  const std::vector<pcl::PointXYZ> input_points = {{0.0F, 0.0F, 0.0F},
                                                   {1.0F, 0.0F, 0.0F},
                                                   {2.0F, 0.0F, 0.0F},
                                                   {3.0F, 0.0F, 0.0F},
                                                   {4.0F, 0.0F, 0.0F}};

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(input_points));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  EXPECT_EQ(xyz_cloud.points.size(), 2U);
  for (const auto &point : xyz_cloud.points) {
    const auto match = std::find_if(input_points.begin(), input_points.end(),
                                    [&point](const pcl::PointXYZ &candidate) {
                                      return candidate.x == point.x &&
                                             candidate.y == point.y &&
                                             candidate.z == point.z;
                                    });
    EXPECT_TRUE(match != input_points.end());
  }

  const auto output_indices =
      blackboard->get<yasmin_pcl::common::Indices>("output_indices");
  EXPECT_EQ(output_indices.size(), 2U);

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 3U);
}
