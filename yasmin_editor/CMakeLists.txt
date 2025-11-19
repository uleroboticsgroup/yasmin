cmake_minimum_required(VERSION 3.5)
project(yasmin_editor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_factory REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  yasmin_editor/yasmin_editor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yasmin_editor
)

ament_package()
