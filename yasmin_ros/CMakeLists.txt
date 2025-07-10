cmake_minimum_required(VERSION 3.5)
project(yasmin_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(SOURCES
  src/yasmin_ros/yasmin_node.cpp
  src/yasmin_ros/ros_logs.cpp
  src/yasmin_ros/action_state.cpp
  src/yasmin_ros/service_state.cpp
  src/yasmin_ros/monitor_state.cpp
  src/yasmin_ros/publisher_state.cpp
  src/yasmin_ros/get_parameters_state.cpp
)
set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  yasmin::yasmin
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  yasmin
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})


# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
install(
  DIRECTORY tests/python/
  DESTINATION pytests/${PROJECT_NAME}
)

install(
  PROGRAMS tests/run_pytests.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
