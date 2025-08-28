cmake_minimum_required(VERSION 3.5)
project(yasmin_demos)

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
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  yasmin_viewer::yasmin_viewer
  ${nav_msgs_TARGETS}
  ${example_interfaces_TARGETS}
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  yasmin
  yasmin_ros
  yasmin_viewer
  nav_msgs
  example_interfaces
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# demo
add_executable(yasmin_demo src/yasmin_demo.cpp)
target_link_libraries(yasmin_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  yasmin_demo
  DESTINATION lib/${PROJECT_NAME}
)

# remap demo
add_executable(remap_demo src/remap_demo.cpp)
target_link_libraries(remap_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  remap_demo
  DESTINATION lib/${PROJECT_NAME}
)

# monitor demo
add_executable(monitor_demo src/monitor_demo.cpp)
target_link_libraries(monitor_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  monitor_demo
  DESTINATION lib/${PROJECT_NAME}
)

# publisher demo
add_executable(publisher_demo src/publisher_demo.cpp)
target_link_libraries(publisher_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  publisher_demo
  DESTINATION lib/${PROJECT_NAME}
)

# service client demo
add_executable(service_client_demo src/service_client_demo.cpp)
target_link_libraries(service_client_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  service_client_demo
  DESTINATION lib/${PROJECT_NAME}
)

# action client demo
add_executable(action_client_demo src/action_client_demo.cpp)
target_link_libraries(action_client_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  action_client_demo
  DESTINATION lib/${PROJECT_NAME}
)

# add_two_ints service server
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server PUBLIC ${DEPENDENCIES})
install(TARGETS
  add_two_ints_server
  DESTINATION lib/${PROJECT_NAME}
)

# fibonacci action server
add_executable(fibonacci_action_server src/fibonacci_action_server.cpp)
target_link_libraries(fibonacci_action_server PUBLIC ${DEPENDENCIES})
install(TARGETS
  fibonacci_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# multiple_states_demos
add_executable(multiple_states_demo src/multiple_states_demo.cpp src/foo_state.cpp src/bar_state.cpp)
target_link_libraries(multiple_states_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  multiple_states_demo
  DESTINATION lib/${PROJECT_NAME}
)

# concurrent states demo
add_executable(concurrence_demo src/concurrence_demo.cpp)
target_link_libraries(concurrence_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  concurrence_demo
  DESTINATION lib/${PROJECT_NAME}
)

# parameters states demo
add_executable(parameters_demo src/parameters_demo.cpp)
target_link_libraries(parameters_demo PUBLIC ${DEPENDENCIES})
install(TARGETS
  parameters_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES_PACKAGE})


# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_demos/yasmin_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/remap_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/monitor_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/publisher_demo.py
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

install(PROGRAMS
  yasmin_demos/multiple_states_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/concurrence_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  yasmin_demos/parameters_demo.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
