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
#include <pcl/io/pcd_io.h>

#include <filesystem>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/io/save_pcd_state.hpp"

TEST(SavePcdState, SavesPcdFileFromBlackboardCloud) {
  const auto file_path = yasmin_pcl::test::make_temp_path("save_pcd", ".pcd");

  yasmin_pcl::io::SavePcdState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.set_parameter<std::string>("storage_mode", "binary");
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 4.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");
  EXPECT_TRUE(std::filesystem::exists(file_path));

  pcl::PCLPointCloud2 loaded_cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(file_path.string(), loaded_cloud), 0);
  EXPECT_EQ(yasmin_pcl::test::to_xyz_cloud(loaded_cloud).points.size(), 2U);

  std::filesystem::remove(file_path);
}

TEST(SavePcdState, SavesBinaryCompressedPcdFile) {
  const auto file_path =
      yasmin_pcl::test::make_temp_path("save_pcd_compressed", ".pcd");

  yasmin_pcl::io::SavePcdState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.set_parameter<std::string>("storage_mode", "binary_compressed");
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 4.0F}}));

  EXPECT_EQ(state(blackboard), "succeeded");
  EXPECT_TRUE(std::filesystem::exists(file_path));

  pcl::PCLPointCloud2 loaded_cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(file_path.string(), loaded_cloud), 0);
  EXPECT_EQ(yasmin_pcl::test::to_xyz_cloud(loaded_cloud).points.size(), 2U);

  std::filesystem::remove(file_path);
}

TEST(SavePcdState, AbortsForUnsupportedStorageMode) {
  const auto file_path =
      yasmin_pcl::test::make_temp_path("save_pcd_bad", ".pcd");

  yasmin_pcl::io::SavePcdState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.set_parameter<std::string>("storage_mode", "invalid_mode");
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 4.0F}}));

  EXPECT_EQ(state(blackboard), "aborted");
  EXPECT_FALSE(std::filesystem::exists(file_path));
}
