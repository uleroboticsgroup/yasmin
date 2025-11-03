cmake_minimum_required(VERSION 3.16)
project(yasmin_factory)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yasmin REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD 17)

# Build pybind11 bridge
pybind11_add_module(yasmin_pybind_bridge 
  src/pybind_bridge.cpp
)
target_link_libraries(yasmin_pybind_bridge PUBLIC
  pluginlib::pluginlib
  yasmin::yasmin
)
install(TARGETS yasmin_pybind_bridge
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
ament_environment_hooks(PYTHONPATH.sh.in)

# Install Python loader package
ament_python_install_package(${PROJECT_NAME})

ament_package()