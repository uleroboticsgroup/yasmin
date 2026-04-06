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
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

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
  src/yasmin_ros/tf_buffer_state.cpp
  src/yasmin_ros/supported_interface_serialization.cpp
)
set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
  yasmin::yasmin
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${example_interfaces_TARGETS}
  ${tf2_msgs_TARGETS}
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
  tf2
  tf2_ros
  tf2_msgs
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Separate plugin libraries to avoid namespace collisions
add_library(ros_serialize_cpp_state SHARED src/yasmin_ros/ros_serialize_cpp_state.cpp)
target_include_directories(ros_serialize_cpp_state PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(ros_serialize_cpp_state PUBLIC
  ${PROJECT_NAME}
  yasmin::yasmin
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
)

add_library(ros_deserialize_cpp_state SHARED src/yasmin_ros/ros_deserialize_cpp_state.cpp)
target_include_directories(ros_deserialize_cpp_state PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(ros_deserialize_cpp_state PUBLIC
  ${PROJECT_NAME}
  yasmin::yasmin
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ros_serialize_cpp_state ros_deserialize_cpp_state
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

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
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
    test_ros_serialize_state
    test_ros_deserialize_state
    test_tf_buffer_state
  )

  foreach(_test_name ${_pytest_tests})
    ament_add_pytest_test(${_test_name} test/${_test_name}.py TIMEOUT 300)
  endforeach()

  set(_gtest_tests
    test_action_client_state
    test_service_client_state
    test_monitor_state
    test_publisher_state
    test_tf_buffer_state
  )

  foreach(_test_name ${_gtest_tests})
    ament_add_gtest(${_test_name}_cpp test/${_test_name}.cpp TIMEOUT 300)
    if(TARGET ${_test_name}_cpp)
      target_link_libraries(${_test_name}_cpp
        ${PROJECT_NAME}
        yasmin::yasmin
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
        tf2::tf2
        tf2_ros::tf2_ros
        ${geometry_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${example_interfaces_TARGETS}
        ${tf2_msgs_TARGETS}
      )
    endif()
  endforeach()

  ament_add_gtest(test_ros_serialization_states_cpp test/test_ros_serialization_states.cpp TIMEOUT 300)
  if(TARGET test_ros_serialization_states_cpp)
    target_link_libraries(test_ros_serialization_states_cpp
      ${PROJECT_NAME}
      ros_serialize_cpp_state
      ros_deserialize_cpp_state
      yasmin::yasmin
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      tf2::tf2
      tf2_ros::tf2_ros
      ${geometry_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
      ${example_interfaces_TARGETS}
      ${tf2_msgs_TARGETS}
    )
  endif()

endif()

ament_package()
