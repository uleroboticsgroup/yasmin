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
      .def("validate", &yasmin::OrthogonalState::validate,
           "Recursively validate region state machines",
           py::arg("strict_mode") = false)
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
