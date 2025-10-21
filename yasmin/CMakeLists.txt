cmake_minimum_required(VERSION 3.5)
project(yasmin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# C++
include_directories(src)
include_directories(include)

set(SOURCES
  src/yasmin/blackboard/blackboard.cpp
  src/yasmin/logs.cpp
  src/yasmin/state.cpp
  src/yasmin/cb_state.cpp
  src/yasmin/state_machine.cpp
  src/yasmin/concurrence.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})


# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  # Python tests
  set(_pytest_tests
    test_blackboard
    test_cb_state
    test_state
    test_state_machine
    test_concurrence
  )
  
  foreach(_test_name ${_pytest_tests})
    ament_add_pytest_test(${_test_name} test/${_test_name}.py)
    ament_add_gtest(${_test_name}_cpp test/${_test_name}.cpp)
    target_link_libraries(${_test_name}_cpp ${PROJECT_NAME})
  endforeach()

endif()

ament_package()
