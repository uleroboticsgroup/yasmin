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
#include "yasmin/concurrence.hpp"
#include "yasmin/pybind11_utils.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::BlackboardPyWrapper);

PYBIND11_MODULE(concurrence, m) {
  m.doc() = "Python bindings for yasmin::Concurrence";

  // Export Concurrence class - inherits from State
  py::class_<yasmin::Concurrence,
             yasmin::State, // Inherit from State
             yasmin::Concurrence::SharedPtr>
      concurrence_class(m, "Concurrence");

  concurrence_class
      .def(py::init<yasmin::StateMap, std::string, yasmin::OutcomeMap,
                    yasmin::ParameterMappingsMap>(),
           py::arg("states"), py::arg("default_outcome"),
           py::arg("outcome_map") = yasmin::OutcomeMap(),
           py::arg("parameter_mappings") = yasmin::ParameterMappingsMap(),
           py::keep_alive<1, 2>(), // Keep states (arg 2) alive as long as self
                                   // (arg 1) is alive
           "Construct a Concurrence state with states, default outcome, "
           "outcome map, and parameter mappings")
      // Getters for states, outcome_map, and default_outcome
      .def("get_states", &yasmin::Concurrence::get_states,
           "Get all states in the concurrence",
           py::return_value_policy::reference_internal)
      .def("get_outcome_map", &yasmin::Concurrence::get_outcome_map,
           "Get the outcome map for this concurrence state",
           py::return_value_policy::reference_internal)
      .def("get_default_outcome", &yasmin::Concurrence::get_default_outcome,
           "Get the default outcome for this concurrence state")

      .def("set_parameter_mappings",
           &yasmin::Concurrence::set_parameter_mappings,
           "Set parameter mappings for a child state", py::arg("state_name"),
           py::arg("parameter_mappings"))
      .def("get_parameter_mappings",
           &yasmin::Concurrence::get_parameter_mappings,
           "Get parameter mappings for all child states",
           py::return_value_policy::reference_internal)
      .def("configure", &yasmin::Concurrence::configure,
           "Configure the concurrence and all child states")
      .def("validate", &yasmin::Concurrence::validate,
           "Recursively validate nested state machines inside this concurrence",
           py::arg("strict_mode") = false)
      // Cancel state method
      .def("cancel_state", &yasmin::Concurrence::cancel_state,
           "Cancel the current state execution")
      // String representation
      .def("to_string", &yasmin::Concurrence::to_string,
           "Convert the concurrence to a string representation")
      .def("__str__", &yasmin::Concurrence::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(concurrence_class),
                                            yasmin::Concurrence>(
      concurrence_class);
}
