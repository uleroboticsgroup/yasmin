cmake_minimum_required(VERSION 3.10)
project(yasmin)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find Python and pybind11 for Python bindings
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Set Python install directory
if(NOT PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# C++
include_directories(src)
include_directories(include)

set(SOURCES
  src/yasmin/blackboard.cpp
  src/yasmin/callback_signal.cpp
  src/yasmin/logs.cpp
  src/yasmin/state.cpp
  src/yasmin/cb_state.cpp
  src/yasmin/state_machine.cpp
  src/yasmin/concurrence.cpp
  src/yasmin/region_barrier.cpp
  src/yasmin/join_state.cpp
  src/yasmin/orthogonal_state.cpp
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


function(add_yasmin_pybind11 MODULE_NAME)
  pybind11_add_module(${MODULE_NAME}
    src/yasmin/${MODULE_NAME}_pybind11.cpp
  )
  target_link_libraries(${MODULE_NAME} PRIVATE ${PROJECT_NAME})
  target_include_directories(${MODULE_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  install(TARGETS ${MODULE_NAME}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

add_yasmin_pybind11(blackboard)
add_yasmin_pybind11(callback_signal)
add_yasmin_pybind11(state)
add_yasmin_pybind11(state_machine)
add_yasmin_pybind11(concurrence)
add_yasmin_pybind11(cb_state)
add_yasmin_pybind11(join_state)
add_yasmin_pybind11(orthogonal_state)
add_yasmin_pybind11(logs)

# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  # Python tests
  set(_pytest_tests
    test_blackboard
    test_callback_signal
    test_cb_state
    test_state
    test_state_machine
    test_concurrence
  test_orthogonal_state
  )
  
  foreach(_test_name ${_pytest_tests})
    ament_add_pytest_test(${_test_name} test/${_test_name}.py)
    ament_add_gtest(${_test_name}_cpp test/${_test_name}.cpp)
    target_link_libraries(${_test_name}_cpp ${PROJECT_NAME})
  endforeach()

endif()

ament_package()
