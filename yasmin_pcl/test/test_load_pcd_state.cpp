// Copyright (C) 2026 Maik Knof

#include <filesystem>

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

#include "test_utils.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/io/load_pcd_state.hpp"

TEST(LoadPcdState, LoadsCloudAndMetadataFromPcdFile) {
  const auto file_path = yasmin_pcl::test::make_temp_path("load_pcd", ".pcd");
  const auto input_cloud = yasmin_pcl::test::create_pcl_cloud_ptr(
      {{0.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 2.0F}});

  pcl::PCDWriter writer;
  const Eigen::Vector4f origin(1.0F, 2.0F, 3.0F, 0.0F);
  const Eigen::Quaternionf orientation(1.0F, 0.0F, 0.0F, 0.0F);
  ASSERT_EQ(
      writer.writeBinary(file_path.string(), *input_cloud, origin, orientation),
      0);

  yasmin_pcl::io::LoadPcdState state;
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
  EXPECT_FLOAT_EQ(sensor_origin[0], 1.0F);
  EXPECT_FLOAT_EQ(sensor_origin[1], 2.0F);
  EXPECT_FLOAT_EQ(sensor_origin[2], 3.0F);

  EXPECT_GE(blackboard->get<int>("pcd_version"), 0);

  std::filesystem::remove(file_path);
}
