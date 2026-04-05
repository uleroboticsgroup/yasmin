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

#include <filesystem>

#include <gtest/gtest.h>

#include <pcl/io/ply_io.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/io/save_ply_state.hpp"

TEST(SavePlyState, SavesPlyFileFromBlackboardCloud) {
  const auto file_path = yasmin_pcl::test::make_temp_path("save_ply", ".ply");

  yasmin_pcl::io::SavePlyState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.set_parameter<bool>("binary_mode", false);
  state.set_parameter<bool>("use_camera", false);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{-1.0F, 0.0F, 0.0F}, {0.5F, 1.5F, 2.5F}}));

  EXPECT_EQ(state(blackboard), "succeeded");
  EXPECT_TRUE(std::filesystem::exists(file_path));

  pcl::PCLPointCloud2 loaded_cloud;
  ASSERT_EQ(pcl::io::loadPLYFile(file_path.string(), loaded_cloud), 0);
  EXPECT_EQ(yasmin_pcl::test::to_xyz_cloud(loaded_cloud).points.size(), 2U);

  std::filesystem::remove(file_path);
}

TEST(SavePlyState, SavesBinaryPlyFileFromBlackboardCloud) {
  const auto file_path =
      yasmin_pcl::test::make_temp_path("save_ply_binary", ".ply");

  yasmin_pcl::io::SavePlyState state;
  state.set_parameter<std::string>("file_path", file_path.string());
  state.set_parameter<bool>("binary_mode", true);
  state.set_parameter<bool>("use_camera", true);
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{-1.0F, 0.0F, 0.0F}, {0.5F, 1.5F, 2.5F}}));

  EXPECT_EQ(state(blackboard), "succeeded");
  EXPECT_TRUE(std::filesystem::exists(file_path));

  pcl::PCLPointCloud2 loaded_cloud;
  ASSERT_EQ(pcl::io::loadPLYFile(file_path.string(), loaded_cloud), 0);
  EXPECT_EQ(yasmin_pcl::test::to_xyz_cloud(loaded_cloud).points.size(), 2U);

  std::filesystem::remove(file_path);
}

TEST(SavePlyState, AbortsWhenFilePathIsEmpty) {
  yasmin_pcl::io::SavePlyState state;
  state.configure();

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<yasmin_pcl::common::PclPointCloud2Ptr>(
      "input_cloud", yasmin_pcl::test::create_pcl_cloud_ptr(
                         {{-1.0F, 0.0F, 0.0F}, {0.5F, 1.5F, 2.5F}}));

  EXPECT_EQ(state(blackboard), "aborted");
}
