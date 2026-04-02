cmake_minimum_required(VERSION 3.5)
project(yasmin_ros)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yasmin REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
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
  src/yasmin_ros/supported_interface_serialization.cpp
  src/yasmin_ros/ros_serialize_cpp_state.cpp
  src/yasmin_ros/ros_deserialize_cpp_state.cpp
)
set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  yasmin::yasmin
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${example_interfaces_TARGETS}
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  pluginlib
  yasmin
  geometry_msgs
  std_msgs
  std_srvs
  example_interfaces
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  yasmin
  geometry_msgs
  std_msgs
  std_srvs
  example_interfaces
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY state_machines/
  DESTINATION share/${PROJECT_NAME}/state_machines
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})

pluginlib_export_plugin_description_file(yasmin plugins.xml)

# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  # Python tests
  set(_pytest_tests
    test_action_client_state
    test_service_client_state
    test_monitor_state
    test_publisher_state
  )

  foreach(_test_name ${_pytest_tests})
    ament_add_pytest_test(${_test_name} test/${_test_name}.py TIMEOUT 300)
    ament_add_gtest(${_test_name}_cpp test/${_test_name}.cpp TIMEOUT 300)
    target_link_libraries(${_test_name}_cpp
      ${PROJECT_NAME}
      yasmin::yasmin
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      ${std_msgs_TARGETS}
      ${example_interfaces_TARGETS}
    )
    ament_target_dependencies(${_test_name}_cpp
      rclcpp
      rclcpp_action
      yasmin
      std_msgs
      example_interfaces
    )
  endforeach()

endif()

ament_package()
