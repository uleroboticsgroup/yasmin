// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YASMIN_PYBIND11_UTILS_HPP_
#define YASMIN_PYBIND11_UTILS_HPP_

#include <pybind11/pybind11.h>

#include <memory>

#include "yasmin/blackboard.hpp"
#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/types.hpp"

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
 * @return yasmin::Blackboard::SharedPtr The C++ blackboard
 */
inline Blackboard::SharedPtr
convert_blackboard_from_python(const py::object &blackboard_obj) {
  Blackboard::SharedPtr blackboard;

  // Case 1: None or not provided - create new Blackboard
  if (blackboard_obj.is_none()) {
    blackboard = yasmin::Blackboard::make_shared();
  }
  // Case 2: Check if it's a BlackboardPyWrapper
  else if (py::isinstance<yasmin::BlackboardPyWrapper>(blackboard_obj)) {
    auto &wrapper = blackboard_obj.cast<yasmin::BlackboardPyWrapper &>();
    // Get the shared pointer directly instead of copying
    blackboard = wrapper.get_cpp_blackboard();
  }
  // Case 3: Check if it's a Blackboard
  else if (py::isinstance<yasmin::Blackboard>(blackboard_obj)) {
    blackboard = blackboard_obj.cast<Blackboard::SharedPtr>();
  }
  // Case 4: Unknown type - create a new blackboard
  else {
    blackboard = yasmin::Blackboard::make_shared();
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
  return [cb](Blackboard::SharedPtr blackboard, auto... args) {
    py::gil_scoped_acquire acquire;
    yasmin::BlackboardPyWrapper wrapper(blackboard);
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
  return [cb](Blackboard::SharedPtr blackboard) -> ReturnType {
    py::gil_scoped_acquire acquire;
    yasmin::BlackboardPyWrapper wrapper(blackboard);
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

/**
 * @brief Per-thread slot for the GIL state saved by the default fork/join
 * hooks.
 *
 * A container's fork and join hooks always run back-to-back on the same
 * thread, so a single per-thread slot is always balanced.
 */
inline PyThreadState *&default_gil_saved_state() {
  thread_local PyThreadState *state = nullptr;
  return state;
}

/**
 * @brief Default before-fork GIL hook.
 *
 * Releases the GIL only if this thread actually holds it (worker threads
 * spawned by OrthogonalState/Concurrence never acquired it, and Python
 * callers may have already released it via add_call_operator).
 */
inline void default_gil_before_fork() {
  if (Py_IsInitialized() && PyGILState_Check()) {
    default_gil_saved_state() = PyEval_SaveThread();
  }
}

/**
 * @brief Default after-join GIL hook.
 *
 * Restores the thread state saved by default_gil_before_fork, if any.
 */
inline void default_gil_after_join() {
  if (default_gil_saved_state()) {
    PyEval_RestoreThread(default_gil_saved_state());
    default_gil_saved_state() = nullptr;
  }
}

/**
 * @brief Registers the default GIL fork/join hooks on a container class.
 *
 * Each binding module calls this for its own container type
 * (OrthogonalState, Concurrence) so that container-specific headers are
 * only needed where the container is actually bound.
 *
 * @tparam Container The container class exposing set_thread_hooks.
 */
template <typename Container> inline void register_default_gil_hooks() {
  Container::set_thread_hooks(default_gil_before_fork, default_gil_after_join);
}

} // namespace pybind11_utils
} // namespace yasmin

#endif // YASMIN_PYBIND11_UTILS_HPP_
