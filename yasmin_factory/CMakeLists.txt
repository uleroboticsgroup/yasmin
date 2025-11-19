cmake_minimum_required(VERSION 3.16)
project(yasmin_factory)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_viewer REQUIRED)
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

# factory node
set(DEPENDENCIES
  rclcpp::rclcpp
  yasmin::yasmin
  yasmin_viewer::yasmin_viewer
  yasmin_factory
)
add_executable(yasmin_factory_node src/yasmin_factory_node.cpp)
target_link_libraries(yasmin_factory_node PUBLIC ${DEPENDENCIES})
install(TARGETS
  yasmin_factory_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python loader package
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  yasmin_factory/yasmin_factory_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Build test plugin library
  add_library(yasmin_factory_test_states SHARED
    test/test_simple_state.cpp
  )
  target_link_libraries(yasmin_factory_test_states PUBLIC
    yasmin::yasmin
    pluginlib::pluginlib
  )
  
  # Install test plugin library
  install(TARGETS yasmin_factory_test_states
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  # Install test plugin description
  install(FILES test/test_plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )
  
  # Install test XML files
  install(FILES
    test/test_simple_sm_1.xml
    test/test_simple_sm_2.xml
    test/test_nested_sm.xml
    test/test_remapping.xml
    DESTINATION share/${PROJECT_NAME}/test
  )

  # Export plugin description for tests
  pluginlib_export_plugin_description_file(yasmin test/test_plugins.xml)

  # Python tests
  ament_add_pytest_test(test_yasmin_factory test/test_yasmin_factory.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # C++ tests
  ament_add_gtest(test_yasmin_factory_cpp test/test_yasmin_factory.cpp)
  target_link_libraries(test_yasmin_factory_cpp
    ${PROJECT_NAME}
    yasmin::yasmin
  )
  target_include_directories(test_yasmin_factory_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

endif()

ament_package()