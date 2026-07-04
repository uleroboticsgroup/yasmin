// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN__CALLBACK_SIGNAL_PYUTILS_HPP_
#define YASMIN__CALLBACK_SIGNAL_PYUTILS_HPP_

#include <pybind11/pybind11.h>

#include <memory>
#include <stdexcept>

#include "yasmin/callback_signal.hpp"

namespace py = pybind11;

namespace yasmin {
namespace callback_signal_pyutils {

/**
 * @brief Check whether a Python object is an instance of yasmin.CallbackSignal.
 * @param value Python object to inspect.
 * @return True if the object is a CallbackSignal instance.
 *
 * The check uses the exported Python class instead of relying on a holder-type
 * based pybind11 isinstance check. This keeps the detection stable across the
 * Python/C++ module boundary where the underlying object is bound with
 * std::shared_ptr ownership.
 */
inline bool is_python_callback_signal_like(const py::handle &value) {
  if (value.is_none()) {
    return false;
  }

  try {
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
    py::module_ callback_signal_module =
        py::module_::import("yasmin.callback_signal");
#else
    py::module callback_signal_module =
        py::module::import("yasmin.callback_signal");
#endif
    py::object callback_signal_class =
        callback_signal_module.attr("CallbackSignal");
    const int is_instance =
        PyObject_IsInstance(value.ptr(), callback_signal_class.ptr());

    if (is_instance == 1) {
      return true;
    }

    if (is_instance == -1) {
      PyErr_Clear();
    }
  } catch (const py::error_already_set &) {
    PyErr_Clear();
  }

  return false;
}

/**
 * @brief Convert a Python CallbackSignal object into its native shared pointer.
 * @param value Python object expected to hold a yasmin.CallbackSignal instance.
 * @return Shared pointer to the native CallbackSignal.
 * @throws std::runtime_error if the conversion fails.
 */
inline CallbackSignal::SharedPtr
cast_python_callback_signal(const py::object &value) {
  try {
    return value.cast<CallbackSignal::SharedPtr>();
  } catch (const py::cast_error &) {
    throw std::runtime_error("Failed to convert Python CallbackSignal to "
                             "std::shared_ptr<yasmin::CallbackSignal>");
  }
}

} // namespace callback_signal_pyutils
} // namespace yasmin

#endif // YASMIN__CALLBACK_SIGNAL_PYUTILS_HPP_
