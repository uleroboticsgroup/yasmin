cmake_minimum_required(VERSION 3.16)
project(yasmin_factory)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yasmin REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(TinyXML2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Build C++ yasmin_factory library
add_library(${PROJECT_NAME} SHARED
  src/yasmin_factory.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Python3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pluginlib::pluginlib
  yasmin::yasmin
  tinyxml2::tinyxml2
  pybind11::embed
  ${Python3_LIBRARIES}
  ${rclcpp_TARGETS}
)

# Export the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  pluginlib
  yasmin
  TinyXML2
  pybind11
)

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