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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
endif()

# C++
include_directories(src)
include_directories(include)

set(SOURCES
  src/yasmin_ros/yasmin_node.cpp
  src/yasmin_ros/ros_logs.cpp
  src/yasmin_ros/ros_clients_cache.cpp
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
if(BUILD_TESTING)
  # Python tests
  ament_add_pytest_test(test_yasmin_ros test/test_yasmin_ros.py)

  # C++ tests
  ament_add_gtest(test_yasmin_ros_cpp test/test_yasmin_ros.cpp)
  target_link_libraries(test_yasmin_ros_cpp ${PROJECT_NAME})
  ament_target_dependencies(test_yasmin_ros_cpp
    rclcpp
    rclcpp_action
    example_interfaces
    std_msgs
    yasmin
  )
endif()

ament_package()
