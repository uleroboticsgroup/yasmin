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

#include <pcl_conversions/pcl_conversions.h>

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
