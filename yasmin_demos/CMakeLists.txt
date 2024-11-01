cmake_minimum_required(VERSION 3.5)
project(yasmin_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "galactic")
  message(STATUS "Build for ROS2 Galactic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGALACTIC")
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Build for ROS2 Humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHUMBLE")
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Build for ROS2 Iron")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIRON")
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Build for ROS2 Jazzy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAZZY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin
  yasmin_ros
  yasmin_viewer
  nav_msgs
  example_interfaces
  action_tutorials_interfaces
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

# service client demo
add_executable(service_client_demo
  src/service_client_demo.cpp
)
set(SOURCES
  src/service_client_demo.cpp
)
ament_target_dependencies(service_client_demo ${DEPENDENCIES})
install(TARGETS
  service_client_demo
  DESTINATION lib/${PROJECT_NAME}
)

# action client demo
add_executable(action_client_demo
  src/action_client_demo.cpp
)
set(SOURCES
  src/action_client_demo.cpp
)
ament_target_dependencies(action_client_demo ${DEPENDENCIES})
install(TARGETS
  action_client_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_demos/yasmin_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/monitor_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/service_client_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/action_client_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/nav_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
