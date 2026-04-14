cmake_minimum_required(VERSION 3.5)
project(yasmin_editor)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_factory REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  yasmin_editor/yasmin_editor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yasmin_editor
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(
    GLOB TEST_FILES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py"
  )

  foreach(TEST_FILE IN LISTS TEST_FILES)
    get_filename_component(TEST_NAME "${TEST_FILE}" NAME_WE)
    ament_add_pytest_test(${TEST_NAME} "${TEST_FILE}")
  endforeach()
endif()

ament_package()
