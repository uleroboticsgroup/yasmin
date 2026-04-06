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
#include <pcl/io/ply_io.h>

#include <Eigen/Geometry>
#include <filesystem>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/io/load_ply_state.hpp"

TEST(LoadPlyState, LoadsCloudAndMetadataFromPlyFile) {
  const auto file_path = yasmin_pcl::test::make_temp_path("load_ply", ".ply");
  const auto input_cloud = yasmin_pcl::test::create_pcl_cloud_ptr(
      {{0.0F, 0.0F, 0.0F}, {1.5F, 0.0F, 0.0F}});
  const Eigen::Vector4f origin(4.0F, 5.0F, 6.0F, 0.0F);
  const Eigen::Quaternionf orientation(1.0F, 0.0F, 0.0F, 0.0F);

  ASSERT_EQ(pcl::io::savePLYFile(file_path.string(), *input_cloud, origin,
                                 orientation, false, true),
            0);

  yasmin_pcl::io::LoadPlyState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), "succeeded");

  const auto output_cloud =
      blackboard->get<yasmin_pcl::common::PclPointCloud2Ptr>("output_cloud");
  ASSERT_TRUE(output_cloud != nullptr);
  EXPECT_EQ(yasmin_pcl::test::to_xyz_cloud(*output_cloud).points.size(), 2U);

  const auto sensor_origin =
      blackboard->get<yasmin_pcl::common::Vector4fArray>("sensor_origin");
  EXPECT_FLOAT_EQ(sensor_origin[0], 4.0F);
  EXPECT_FLOAT_EQ(sensor_origin[1], 5.0F);
  EXPECT_FLOAT_EQ(sensor_origin[2], 6.0F);

  std::filesystem::remove(file_path);
}

TEST(LoadPlyState, AbortsWhenFilePathIsEmpty) {
  yasmin_pcl::io::LoadPlyState state;
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), "aborted");
}

TEST(LoadPlyState, AbortsWhenFileDoesNotExist) {
  yasmin_pcl::io::LoadPlyState state;
  state.set_parameter<std::string>("file_path",
                                   "/tmp/yasmin_pcl_missing_input_file.ply");
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), "aborted");
}
