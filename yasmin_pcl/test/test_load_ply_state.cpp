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
