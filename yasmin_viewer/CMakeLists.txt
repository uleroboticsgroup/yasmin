cmake_minimum_required(VERSION 3.5)
project(yasmin_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_msgs REQUIRED)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCES
  src/yasmin_viewer/yasmin_viewer_pub.cpp
)

set(DEPENDENCIES
  rclcpp::rclcpp
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  ${yasmin_msgs_TARGETS}
)

set(DEPENDENCIES_PACKAGE
  rclcpp
  rclpy
  yasmin
  yasmin_ros
  yasmin_msgs
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

add_executable(yasmin_viewer_node
  src/yasmin_viewer/main.cpp
  src/yasmin_viewer/yasmin_viewer_node.cpp
)
target_include_directories(yasmin_viewer_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(yasmin_viewer_node
  ament_index_cpp
  rclcpp
  yasmin_msgs
)
target_link_libraries(yasmin_viewer_node PUBLIC ${Boost_LIBRARIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} yasmin_viewer_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})

ament_package()
