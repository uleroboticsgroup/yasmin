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

#include <pybind11/pybind11.h>

#include <memory>
#include <utility>

#include "yasmin/callback_signal.hpp"

namespace py = pybind11;

namespace {

class PythonCallbackHolder {
public:
  explicit PythonCallbackHolder(py::function callback)
      : callback_(std::move(callback)) {}

  ~PythonCallbackHolder() {
    if (Py_IsInitialized() == 0) {
      return;
    }

    py::gil_scoped_acquire acquire;
    this->callback_ = py::none();
  }

  void invoke() {
    py::gil_scoped_acquire acquire;

    try {
      this->callback_();
    } catch (const py::error_already_set &error) {
      throw std::runtime_error(py::str(error.value()).cast<std::string>());
    }
  }

private:
  py::object callback_;
};

} // namespace

PYBIND11_MODULE(callback_signal, m) {
  m.doc() = "Python bindings for yasmin::CallbackSignal";

  py::class_<yasmin::CallbackSignalFuture,
             yasmin::CallbackSignalFuture::SharedPtr>(m, "CallbackSignalFuture")
      .def("wait", &yasmin::CallbackSignalFuture::wait,
           py::call_guard<py::gil_scoped_release>(),
           "Wait until the asynchronous trigger completes")
      .def("is_completed", &yasmin::CallbackSignalFuture::is_completed,
           "Check whether the asynchronous trigger completed")
      .def("has_exception", &yasmin::CallbackSignalFuture::has_exception,
           "Check whether a callback exception was captured")
      .def("get_exception_message",
           &yasmin::CallbackSignalFuture::get_exception_message,
           "Get the captured exception message, if any");

  py::class_<yasmin::CallbackSignal, yasmin::CallbackSignal::SharedPtr>(
      m, "CallbackSignal")
      .def(py::init<>())
      .def(
          "add_callback",
          [](yasmin::CallbackSignal &self, py::function callback) {
            auto callback_holder =
                std::make_shared<PythonCallbackHolder>(std::move(callback));
            return self.add_callback(
                [callback_holder]() { callback_holder->invoke(); });
          },
          "Register a Python callback and return its identifier",
          py::arg("callback"))
      .def("add_cancel_callback", &yasmin::CallbackSignal::add_cancel_callback,
           "Register a callback that cancels the provided state",
           py::arg("state"))
      .def("remove_callback", &yasmin::CallbackSignal::remove_callback,
           "Remove a callback by identifier", py::arg("callback_id"))
      .def("clear_callbacks", &yasmin::CallbackSignal::clear_callbacks,
           "Remove all registered callbacks")
      .def("callback_count", &yasmin::CallbackSignal::callback_count,
           "Get the number of registered callbacks")
      .def("empty", &yasmin::CallbackSignal::empty,
           "Check whether the signal has no callbacks")
      .def("trigger", &yasmin::CallbackSignal::trigger,
           py::call_guard<py::gil_scoped_release>(),
           "Trigger all registered callbacks synchronously")
      .def("trigger_async", &yasmin::CallbackSignal::trigger_async,
           "Trigger all registered callbacks asynchronously");
}
