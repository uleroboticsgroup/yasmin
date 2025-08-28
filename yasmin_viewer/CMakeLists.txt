cmake_minimum_required(VERSION 3.5)
project(yasmin_viewer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_msgs REQUIRED)

# C++
include_directories(src)
include_directories(include)

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

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY yasmin_viewer_web_client/build/
  DESTINATION share/yasmin_viewer/yasmin_viewer_web_client/
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})


# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_viewer/yasmin_viewer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yasmin_viewer_node
)

ament_package()
