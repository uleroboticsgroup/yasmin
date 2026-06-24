// Copyright (C) 2026 Miguel Ángel González Santamarta
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
