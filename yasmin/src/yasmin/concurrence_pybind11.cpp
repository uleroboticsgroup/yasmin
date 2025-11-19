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
#include "yasmin/concurrence.hpp"
#include "yasmin/pybind11_utils.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::blackboard::BlackboardPyWrapper);

PYBIND11_MODULE(concurrence, m) {
  m.doc() = "Python bindings for yasmin::Concurrence";

  // Export Concurrence class - inherits from State
  py::class_<yasmin::Concurrence,
             yasmin::State, // Inherit from State
             std::shared_ptr<yasmin::Concurrence>>
      concurrence_class(m, "Concurrence");

  concurrence_class
      .def(py::init<std::map<std::string, std::shared_ptr<yasmin::State>>,
                    std::string, yasmin::Concurrence::OutcomeMap>(),
           py::arg("states"), py::arg("default_outcome"),
           py::arg("outcome_map") = yasmin::Concurrence::OutcomeMap(),
           py::keep_alive<1, 2>()) // Keep states (arg 2) alive as long as self
                                   // (arg 1) is alive
      .def("get_states", &yasmin::Concurrence::get_states,
           "Get all states in the concurrence",
           py::return_value_policy::reference_internal)
      .def("get_outcome_map", &yasmin::Concurrence::get_outcome_map,
           "Get the outcome map for this concurrence state",
           py::return_value_policy::reference_internal)
      .def("get_default_outcome", &yasmin::Concurrence::get_default_outcome,
           "Get the default outcome for this concurrence state")
      .def("cancel_state", &yasmin::Concurrence::cancel_state,
           "Cancel the current state execution")
      .def("to_string", &yasmin::Concurrence::to_string,
           "Convert the concurrence to a string representation")
      .def("__str__", &yasmin::Concurrence::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(concurrence_class),
                                            yasmin::Concurrence>(
      concurrence_class);
}
