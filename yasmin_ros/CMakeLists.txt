cmake_minimum_required(VERSION 3.5)
project(yasmin_ros)

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
find_package(simple_node REQUIRED)
find_package(yasmin REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/yasmin_ros/action_state.cpp
  src/yasmin_ros/service_state.cpp
)

set(DEPENDENCIES
    rclcpp
    simple_node
    yasmin
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

ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})


# Python
ament_python_install_package(${PROJECT_NAME})

ament_package()
