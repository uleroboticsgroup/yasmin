cmake_minimum_required(VERSION 3.5)
project(yasmin_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(simple_node REQUIRED)
find_package(nav_msgs REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin
  yasmin_ros
  yasmin_viewer
  simple_node
  nav_msgs
)

# demo
add_executable(yasmin_demo
  src/yasmin_demo.cpp
)
set(SOURCES
  src/yasmin_demo.cpp
)
ament_target_dependencies(yasmin_demo ${DEPENDENCIES})
install(TARGETS
  yasmin_demo
  DESTINATION lib/${PROJECT_NAME}
)

# monitor demo
add_executable(monitor_demo
  src/monitor_demo.cpp
)
set(SOURCES
  src/monitor_demo.cpp
)
ament_target_dependencies(monitor_demo ${DEPENDENCIES})
install(TARGETS
  monitor_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_demo/yasmin_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demo/monitor_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
