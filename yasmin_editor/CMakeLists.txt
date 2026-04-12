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
  ament_add_pytest_test(test_editor_gui_helpers test/test_editor_gui_helpers.py)
  ament_add_pytest_test(test_free_position_and_recent_files test/test_free_position_and_recent_files.py)
  ament_add_pytest_test(test_runtime_and_transition_helpers test/test_runtime_and_transition_helpers.py)
  ament_add_pytest_test(test_model_validation test/test_model_validation.py)
endif()

ament_package()
