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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yasmin/blackboard_pywrapper.hpp"

namespace py = pybind11;

// No PYBIND11_MAKE_OPAQUE needed here because BlackboardPyWrapper is
// defined and registered in this module, not just forward-declared

PYBIND11_MODULE(blackboard, m) {
  m.doc() = "Python bindings for yasmin::Blackboard";

#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
  py::module_::import("yasmin.callback_signal");
#else
  py::module::import("yasmin.callback_signal");
#endif

  py::class_<yasmin::BlackboardPyWrapper>(m, "Blackboard")
      .def(py::init<>())
      .def(py::init<const yasmin::BlackboardPyWrapper &>(), py::arg("other"))
      .def(
          "copy",
          [](const yasmin::BlackboardPyWrapper &self) {
            return yasmin::BlackboardPyWrapper(self);
          },
          "Create a blackboard copy that shares values but keeps its own "
          "remappings")
      .def(
          "__copy__",
          [](const yasmin::BlackboardPyWrapper &self) {
            return yasmin::BlackboardPyWrapper(self);
          },
          "Create a blackboard copy that shares values but keeps its own "
          "remappings")
      .def(
          "__deepcopy__",
          [](const yasmin::BlackboardPyWrapper &self, py::object) {
            return yasmin::BlackboardPyWrapper(self);
          },
          "Create a blackboard copy that shares values but keeps its own "
          "remappings",
          py::arg("memo"))
      // Setters using set method and __setitem__/__setattr__
      .def("set", &yasmin::BlackboardPyWrapper::set,
           "Set a value in the blackboard", py::arg("key"), py::arg("value"))
      .def("__setitem__", &yasmin::BlackboardPyWrapper::set,
           "Set a value in the blackboard", py::arg("key"), py::arg("value"))
      .def(
          "__setattr__",
          [](yasmin::BlackboardPyWrapper &self, const std::string &name,
             py::object value) { self.set(name, value); },
          "Set a value in the blackboard using attribute access",
          py::arg("name"), py::arg("value"))
      // Getters using get method and __getitem__/__getattr__
      .def("get", &yasmin::BlackboardPyWrapper::get,
           "Get a value from the blackboard", py::arg("key"))
      .def("__getitem__", &yasmin::BlackboardPyWrapper::get,
           "Get a value from the blackboard", py::arg("key"))
      .def(
          "__getattr__",
          [](yasmin::BlackboardPyWrapper &self,
             const std::string &name) -> py::object { return self.get(name); },
          "Get a value from the blackboard using attribute access",
          py::arg("name"))
      // Remove method and __delitem__
      .def("remove", &yasmin::BlackboardPyWrapper::remove,
           "Remove a value from the blackboard", py::arg("key"))
      .def("__delitem__", &yasmin::BlackboardPyWrapper::remove,
           "Remove a value from the blackboard", py::arg("key"))
      // Contain method and __contains__
      .def("contains", &yasmin::BlackboardPyWrapper::contains,
           "Check if a key exists in the blackboard", py::arg("key"))
      .def("__contains__", &yasmin::BlackboardPyWrapper::contains,
           "Check if a key exists in the blackboard", py::arg("key"))
      // Size method and __len__
      .def("size", &yasmin::BlackboardPyWrapper::size,
           "Get the number of key-value pairs in the blackboard")
      .def("__len__", &yasmin::BlackboardPyWrapper::size,
           "Get the number of key-value pairs in the blackboard")
      // Iteration helpers
      .def("keys", &yasmin::BlackboardPyWrapper::keys,
           "Get the keys visible in the current remapping scope")
      .def("values", &yasmin::BlackboardPyWrapper::values,
           "Get the values visible in the current remapping scope")
      .def("items", &yasmin::BlackboardPyWrapper::items,
           "Get the key-value pairs visible in the current remapping scope")
      .def(
          "__iter__",
          [](const yasmin::BlackboardPyWrapper &self) {
            py::list keys = py::cast(self.keys());
            return keys.attr("__iter__")();
          },
          "Iterate over the keys visible in the current remapping scope")
      // String representation
      .def("to_string", &yasmin::BlackboardPyWrapper::to_string,
           "Convert the blackboard to a string representation")
      .def("__str__", &yasmin::BlackboardPyWrapper::to_string,
           "Convert the blackboard to a string representation")
      // Remappings
      .def("set_remappings", &yasmin::BlackboardPyWrapper::set_remappings,
           "Set the key remappings", py::arg("remappings"))
      .def("get_remappings", &yasmin::BlackboardPyWrapper::get_remappings,
           "Get the key remappings");
}
