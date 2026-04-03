// Copyright (C) 2026 Maik Knof

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
