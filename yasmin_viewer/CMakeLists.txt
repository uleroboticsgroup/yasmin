cmake_minimum_required(VERSION 3.5)
project(yasmin_viewer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_msgs REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/yasmin_viewer/yasmin_viewer_pub.cpp
)

set(DEPENDENCIES
    rclcpp
    yasmin
    yasmin_ros
    yasmin_msgs
)

add_library(${LIB} STATIC ${SOURCES})

ament_target_dependencies(${LIB} ${DEPENDENCIES})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY yasmin_viewer_web_client/build/
  DESTINATION share/yasmin_viewer/yasmin_viewer_web_client/
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})


# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_viewer/yasmin_viewer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yasmin_viewer_node
)

ament_package()
