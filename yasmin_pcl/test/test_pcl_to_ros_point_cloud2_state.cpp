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
#include "yasmin_pcl/io/pcl_to_ros_point_cloud2_state.hpp"

TEST(PclToRosPointCloud2State, ConvertsPclCloudToRosCloud) {
  yasmin_pcl::io::PclToRosPointCloud2State state;
  auto blackboard = yasmin::Blackboard::make_shared();

  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{-1.0F, 0.5F, 2.0F}, {2.0F, 1.0F, 0.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::RosPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*output_cloud, pcl_cloud);
  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(pcl_cloud);

  ASSERT_EQ(xyz_cloud.points.size(), 2U);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].x, -1.0F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].y, 0.5F);
  EXPECT_FLOAT_EQ(xyz_cloud.points[0].z, 2.0F);
}

TEST(PclToRosPointCloud2State, AbortsWhenInputCloudIsNull) {
  yasmin_pcl::io::PclToRosPointCloud2State state;
  auto blackboard = yasmin::Blackboard::make_shared();

  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>("input_cloud",
                                                         nullptr);

  EXPECT_EQ(state(blackboard), "aborted");
}
