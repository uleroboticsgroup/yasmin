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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yasmin/blackboard_pywrapper.hpp"

namespace py = pybind11;

PYBIND11_MODULE(blackboard, m) {
  m.doc() = "Python bindings for yasmin::Blackboard";

  py::class_<yasmin::BlackboardPyWrapper>(m, "Blackboard")
      .def(py::init<>())
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
