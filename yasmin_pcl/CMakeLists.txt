cmake_minimum_required(VERSION 3.5)
project(yasmin_pcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters sample_consensus)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yasmin REQUIRED)

set(YASMIN_PCL_SOURCES
  src/io/ros_to_pcl_point_cloud2_state.cpp
  src/io/pcl_to_ros_point_cloud2_state.cpp
  src/io/load_pcd_state.cpp
  src/io/save_pcd_state.cpp
  src/io/load_ply_state.cpp
  src/io/save_ply_state.cpp
  src/filters/pass_through_state.cpp
  src/filters/crop_box_state.cpp
  src/filters/voxel_grid_state.cpp
  src/filters/extract_indices_state.cpp
  src/filters/statistical_outlier_removal_state.cpp
  src/filters/radius_outlier_removal_state.cpp
  src/filters/random_sample_state.cpp
  src/filters/project_inliers_state.cpp
)

add_library(${PROJECT_NAME}_states SHARED ${YASMIN_PCL_SOURCES})
target_include_directories(${PROJECT_NAME}_states
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_states
  PUBLIC
    ${PCL_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    pluginlib::pluginlib
    ${sensor_msgs_TARGETS}
    yasmin::yasmin
)

pluginlib_export_plugin_description_file(yasmin plugins.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_states
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  pluginlib
  sensor_msgs
  pcl_conversions
  yasmin
)

function(add_yasmin_pcl_gtest target_name source_file)
  ament_add_gtest(${target_name} ${source_file})
  if(TARGET ${target_name})
    target_include_directories(${target_name}
      PRIVATE
        include
        test
        ${PCL_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
    )
    target_link_libraries(${target_name}
      ${PROJECT_NAME}_states
      ${PCL_LIBRARIES}
      ${pcl_conversions_LIBRARIES}
      pluginlib::pluginlib
      ${sensor_msgs_TARGETS}
      yasmin::yasmin
    )
  endif()
endfunction()

add_yasmin_pcl_gtest(test_ros_to_pcl_point_cloud2_state
  test/test_ros_to_pcl_point_cloud2_state.cpp)
add_yasmin_pcl_gtest(test_pcl_to_ros_point_cloud2_state
  test/test_pcl_to_ros_point_cloud2_state.cpp)
add_yasmin_pcl_gtest(test_load_pcd_state test/test_load_pcd_state.cpp)
add_yasmin_pcl_gtest(test_save_pcd_state test/test_save_pcd_state.cpp)
add_yasmin_pcl_gtest(test_load_ply_state test/test_load_ply_state.cpp)
add_yasmin_pcl_gtest(test_save_ply_state test/test_save_ply_state.cpp)
add_yasmin_pcl_gtest(test_pass_through_state test/test_pass_through_state.cpp)
add_yasmin_pcl_gtest(test_crop_box_state test/test_crop_box_state.cpp)
add_yasmin_pcl_gtest(test_voxel_grid_state test/test_voxel_grid_state.cpp)
add_yasmin_pcl_gtest(test_extract_indices_state test/test_extract_indices_state.cpp)
add_yasmin_pcl_gtest(test_statistical_outlier_removal_state test/test_statistical_outlier_removal_state.cpp)
add_yasmin_pcl_gtest(test_radius_outlier_removal_state test/test_radius_outlier_removal_state.cpp)
add_yasmin_pcl_gtest(test_random_sample_state test/test_random_sample_state.cpp)
add_yasmin_pcl_gtest(test_project_inliers_state test/test_project_inliers_state.cpp)

ament_package()
