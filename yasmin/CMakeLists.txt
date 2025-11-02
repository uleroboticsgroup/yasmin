cmake_minimum_required(VERSION 3.5)
project(yasmin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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


# Python bindings for Blackboard
pybind11_add_module(blackboard 
  src/yasmin/blackboard/blackboard_pybind11.cpp
)
target_link_libraries(blackboard PRIVATE ${PROJECT_NAME})
target_include_directories(blackboard PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS blackboard
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Python bindings for State
pybind11_add_module(state
  src/yasmin/state_pybind11.cpp
)
target_link_libraries(state PRIVATE ${PROJECT_NAME})
target_include_directories(state PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS state
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Python bindings for StateMachine
pybind11_add_module(state_machine
  src/yasmin/state_machine_pybind11.cpp
)
target_link_libraries(state_machine PRIVATE ${PROJECT_NAME})
target_include_directories(state_machine PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS state_machine
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Python bindings for Concurrence
pybind11_add_module(concurrence
  src/yasmin/concurrence_pybind11.cpp
)
target_link_libraries(concurrence PRIVATE ${PROJECT_NAME})
target_include_directories(concurrence PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS concurrence
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Python bindings for CbState
pybind11_add_module(cb_state
  src/yasmin/cb_state_pybind11.cpp
)
target_link_libraries(cb_state PRIVATE ${PROJECT_NAME})
target_include_directories(cb_state PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS cb_state
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Python bindings for Logs
pybind11_add_module(logs
  src/yasmin/logs_pybind11.cpp
)
target_link_libraries(logs PRIVATE ${PROJECT_NAME})
target_include_directories(logs PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(TARGETS logs
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

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
