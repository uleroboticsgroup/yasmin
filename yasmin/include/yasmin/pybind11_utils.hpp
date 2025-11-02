// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_PYBIND11_UTILS_HPP
#define YASMIN_PYBIND11_UTILS_HPP

#include <memory>
#include <pybind11/pybind11.h>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_pywrapper.hpp"

namespace py = pybind11;

namespace yasmin {
namespace pybind11_utils {

/**
 * @brief Convert a Python blackboard object to a C++ Blackboard shared pointer.
 *
 * This function handles multiple input types:
 * - None/not provided: creates a new Blackboard
 * - BlackboardPyWrapper: extracts the underlying C++ Blackboard
 * - Blackboard: uses directly
 * - Other types: creates a new Blackboard
 *
 * @param blackboard_obj Python object that may contain a blackboard
 * @return std::shared_ptr<yasmin::blackboard::Blackboard> The C++ blackboard
 */
inline std::shared_ptr<yasmin::blackboard::Blackboard>
convert_blackboard_from_python(py::object blackboard_obj) {
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard;

  // Case 1: None or not provided - create new Blackboard
  if (blackboard_obj.is_none()) {
    blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  }
  // Case 2: Check if it's a BlackboardPyWrapper
  else if (py::isinstance<yasmin::blackboard::BlackboardPyWrapper>(
               blackboard_obj)) {
    auto wrapper =
        blackboard_obj.cast<yasmin::blackboard::BlackboardPyWrapper>();
    // Get the shared pointer directly instead of copying
    blackboard = wrapper.get_cpp_blackboard();
  }
  // Case 3: Check if it's a Blackboard
  else if (py::isinstance<yasmin::blackboard::Blackboard>(blackboard_obj)) {
    blackboard =
        blackboard_obj.cast<std::shared_ptr<yasmin::blackboard::Blackboard>>();
  }
  // Case 4: Unknown type - create a new blackboard
  else {
    blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  }

  return blackboard;
}

/**
 * @brief Wrap a C++ callback to handle BlackboardPyWrapper conversion (void
 * return).
 *
 * This is a generic wrapper for callbacks that take a blackboard as the first
 * parameter and return void.
 *
 * @tparam Func The function type to wrap
 * @param cb The Python callback function
 * @return A wrapped C++ function that converts Blackboard to
 * BlackboardPyWrapper
 */
template <typename Func> inline auto wrap_blackboard_callback(py::function cb) {
  return [cb](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
              auto... args) {
    py::gil_scoped_acquire acquire;
    yasmin::blackboard::BlackboardPyWrapper wrapper(blackboard);
    cb(wrapper, args...);
  };
}

/**
 * @brief Wrap a C++ callback to handle BlackboardPyWrapper conversion with
 * return value.
 *
 * This wrapper is for callbacks that return a value (e.g., std::string for
 * CbState).
 *
 * @tparam ReturnType The return type of the callback
 * @param cb The Python callback function
 * @return A wrapped C++ function that converts Blackboard to
 * BlackboardPyWrapper and returns the callback result
 */
template <typename ReturnType>
inline auto wrap_blackboard_callback_with_return(py::function cb) {
  return [cb](std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
             -> ReturnType {
    py::gil_scoped_acquire acquire;
    yasmin::blackboard::BlackboardPyWrapper wrapper(blackboard);
    return cb(wrapper).cast<ReturnType>();
  };
}

/**
 * @brief Helper to define the standard __call__ method for State classes.
 *
 * This template function adds a __call__ method that:
 * 1. Accepts an optional Python blackboard object
 * 2. Converts it to a C++ Blackboard using convert_blackboard_from_python
 * 3. Releases the GIL to allow C++ threads to run
 * 4. Calls the C++ operator() with the converted blackboard
 * 5. Reacquires the GIL before returning to Python
 *
 * @tparam ClassType The pybind11 class type
 * @tparam StateType The state class type (State, StateMachine, Concurrence)
 * @param cls The pybind11 class definition
 */
template <typename ClassType, typename StateType>
inline void add_call_operator(ClassType &cls) {
  cls.def(
      "__call__",
      [](StateType &self, py::object blackboard_obj = py::none()) {
        auto blackboard = convert_blackboard_from_python(blackboard_obj);
        // Release GIL to allow C++ threads (important for Concurrence) to run
        py::gil_scoped_release release;
        return self(blackboard);
      },
      "Execute the state and return the outcome",
      py::arg("blackboard") = py::none());
}

} // namespace pybind11_utils
} // namespace yasmin

#endif // YASMIN_PYBIND11_UTILS_HPP
