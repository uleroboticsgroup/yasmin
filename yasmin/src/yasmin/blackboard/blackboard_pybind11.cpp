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

#include "yasmin/blackboard/blackboard_pywrapper.hpp"

namespace py = pybind11;

PYBIND11_MODULE(blackboard, m) {
  m.doc() = "Python bindings for yasmin::blackboard::Blackboard";

  py::class_<yasmin::blackboard::BlackboardPyWrapper>(m, "Blackboard")
      .def(py::init<>())
      .def("set", &yasmin::blackboard::BlackboardPyWrapper::set,
           "Set a value in the blackboard", py::arg("key"), py::arg("value"))
      .def("__setitem__", &yasmin::blackboard::BlackboardPyWrapper::set,
           "Set a value in the blackboard", py::arg("key"), py::arg("value"))
      .def(
          "__setattr__",
          [](yasmin::blackboard::BlackboardPyWrapper &self,
             const std::string &name,
             py::object value) { self.set(name, value); },
          "Set a value in the blackboard using attribute access",
          py::arg("name"), py::arg("value"))
      .def("get", &yasmin::blackboard::BlackboardPyWrapper::get,
           "Get a value from the blackboard", py::arg("key"))
      .def("__getitem__", &yasmin::blackboard::BlackboardPyWrapper::get,
           "Get a value from the blackboard", py::arg("key"))
      .def(
          "__getattr__",
          [](yasmin::blackboard::BlackboardPyWrapper &self,
             const std::string &name) -> py::object { return self.get(name); },
          "Get a value from the blackboard using attribute access",
          py::arg("name"))
      .def("remove", &yasmin::blackboard::BlackboardPyWrapper::remove,
           "Remove a value from the blackboard", py::arg("key"))
      .def("__delitem__", &yasmin::blackboard::BlackboardPyWrapper::remove,
           "Remove a value from the blackboard", py::arg("key"))
      .def("contains", &yasmin::blackboard::BlackboardPyWrapper::contains,
           "Check if a key exists in the blackboard", py::arg("key"))
      .def("__contains__", &yasmin::blackboard::BlackboardPyWrapper::contains,
           "Check if a key exists in the blackboard", py::arg("key"))
      .def("size", &yasmin::blackboard::BlackboardPyWrapper::size,
           "Get the number of key-value pairs in the blackboard")
      .def("__len__", &yasmin::blackboard::BlackboardPyWrapper::size,
           "Get the number of key-value pairs in the blackboard")
      .def("to_string", &yasmin::blackboard::BlackboardPyWrapper::to_string,
           "Convert the blackboard to a string representation")
      .def("__str__", &yasmin::blackboard::BlackboardPyWrapper::to_string,
           "Convert the blackboard to a string representation")
      .def("set_remappings",
           &yasmin::blackboard::BlackboardPyWrapper::set_remappings,
           "Set the key remappings", py::arg("remappings"))
      .def("get_remappings",
           &yasmin::blackboard::BlackboardPyWrapper::get_remappings,
           "Get the key remappings");
}
