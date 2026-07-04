// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include "yasmin/join_state.hpp"
#include "yasmin/pybind11_utils.hpp"

namespace py = pybind11;

PYBIND11_MODULE(join_state, m) {
  m.doc() = "Python bindings for yasmin::JoinState";

  py::class_<yasmin::JoinState, yasmin::State, yasmin::JoinState::SharedPtr>
      join_state_class(m, "JoinState");

  join_state_class.def(py::init<>())
      .def(py::init<const std::string &, const std::string &>(),
           py::arg("sync_id"), py::arg("outcome") = "joined")
      .def("get_sync_id", &yasmin::JoinState::get_sync_id,
           "Get the sync ID for this JoinState")
      .def("to_string", &yasmin::JoinState::to_string,
           "Convert to string representation")
      .def("__str__", &yasmin::JoinState::to_string);

  yasmin::pybind11_utils::add_call_operator<decltype(join_state_class),
                                            yasmin::JoinState>(
      join_state_class);
}
