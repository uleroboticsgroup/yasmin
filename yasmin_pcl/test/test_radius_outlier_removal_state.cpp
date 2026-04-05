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
#include "yasmin_pcl/filters/radius_outlier_removal_state.hpp"

TEST(RadiusOutlierRemovalState, RemovesIsolatedPoint) {
  yasmin_pcl::filters::RadiusOutlierRemovalState state;
  state.set_parameter<double>("radius_search", 0.2);
  state.set_parameter<int>("min_neighbors_in_radius", 2);
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
                                              {1.00F, 1.00F, 1.00F}}));

  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);

  const auto xyz_cloud = yasmin_pcl::test::to_xyz_cloud(*output_cloud);
  EXPECT_EQ(xyz_cloud.points.size(), 5U);
  for (const auto &point : xyz_cloud.points) {
    EXPECT_LT(point.x, 0.2F);
  }

  const auto removed_indices =
      blackboard->get<yasmin_pcl::common::Indices>("removed_indices");
  EXPECT_EQ(removed_indices.size(), 1U);
}

TEST(RadiusOutlierRemovalState, AbortsWhenInputCloudIsNull) {
  yasmin_pcl::filters::RadiusOutlierRemovalState state;
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>("input_cloud",
                                                         nullptr);

  EXPECT_EQ(state(blackboard), "aborted");
}
