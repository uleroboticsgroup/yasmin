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

#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/pybind11_utils.hpp"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(yasmin::BlackboardPyWrapper);

namespace {

PyThreadState *default_gil_state = nullptr;

void default_gil_before_fork() {
  if (Py_IsInitialized()) {
    // Only release the GIL if this thread actually holds it.
    // add_call_operator may have already released it via gil_scoped_release.
    if (PyGILState_Check()) {
      default_gil_state = PyEval_SaveThread();
    } else {
      default_gil_state = nullptr;
    }
  }
}

void default_gil_after_join() {
  if (default_gil_state) {
    PyEval_RestoreThread(default_gil_state);
    default_gil_state = nullptr;
  }
}

} // namespace

PYBIND11_MODULE(orthogonal_state, m) {
  m.doc() = "Python bindings for yasmin::OrthogonalState";

  py::class_<yasmin::OrthogonalState::RegionDescriptor> region_descriptor(
      m, "RegionDescriptor");
  region_descriptor
      .def_readonly("name", &yasmin::OrthogonalState::RegionDescriptor::name)
      .def_readonly("sm", &yasmin::OrthogonalState::RegionDescriptor::sm);

  py::class_<yasmin::OrthogonalState, yasmin::State,
             yasmin::OrthogonalState::SharedPtr>
      orthogonal_state_class(m, "OrthogonalState");

  orthogonal_state_class
      .def(py::init<const std::string &, const yasmin::OutcomeMap &>(),
           py::arg("default_outcome"),
           py::arg("outcome_map") = yasmin::OutcomeMap())
      .def("add_region", &yasmin::OrthogonalState::add_region, py::arg("name"),
           py::arg("sm"), py::keep_alive<1, 3>())
      .def("get_regions", &yasmin::OrthogonalState::get_regions,
           py::return_value_policy::reference_internal)
      .def("get_outcome_map", &yasmin::OrthogonalState::get_outcome_map,
           py::return_value_policy::reference_internal)
      .def("get_default_outcome", &yasmin::OrthogonalState::get_default_outcome)
      .def("configure", &yasmin::OrthogonalState::configure)
      .def("cancel_state", &yasmin::OrthogonalState::cancel_state)
      .def("to_string", &yasmin::OrthogonalState::to_string)
      .def("__str__", &yasmin::OrthogonalState::to_string);

  yasmin::pybind11_utils::add_call_operator<decltype(orthogonal_state_class),
                                            yasmin::OrthogonalState>(
      orthogonal_state_class);

  m.def("setup_default_gil_hooks", []() {
    yasmin::OrthogonalState::set_thread_hooks(default_gil_before_fork,
                                              default_gil_after_join);
  });
}
