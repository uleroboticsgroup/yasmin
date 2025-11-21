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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yasmin/blackboard/blackboard_pywrapper.hpp"
#include "yasmin/pybind11_utils.hpp"
#include "yasmin/state_machine.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::blackboard::BlackboardPyWrapper);

PYBIND11_MODULE(state_machine, m) {
  m.doc() = "Python bindings for yasmin::StateMachine";

  // Export StateMachine class - inherits from State
  py::class_<yasmin::StateMachine,
             yasmin::State, // Inherit from State
             std::shared_ptr<yasmin::StateMachine>>
      sm_class(m, "StateMachine");

  sm_class.def(py::init<std::set<std::string>>(), py::arg("outcomes"))
      .def(py::init([](const std::vector<std::string> &outcomes) {
             return new yasmin::StateMachine(
                 std::set<std::string>(outcomes.begin(), outcomes.end()));
           }),
           py::arg("outcomes"))
      // Add destructor from StateMachine
      .def("__del__", [](yasmin::StateMachine *self) { delete self; })
      .def(
          "add_state",
          [](yasmin::StateMachine &self, const std::string &name,
             std::shared_ptr<yasmin::State> state,
             const std::map<std::string, std::string> &transitions,
             const std::map<std::string, std::string> &remappings) {
            // Ensure the Python object is kept alive
            py::object py_state = py::cast(state);
            self.add_state(name, state, transitions, remappings);
          },
          "Add a state to the state machine", py::arg("name"), py::arg("state"),
          py::arg("transitions") = std::map<std::string, std::string>(),
          py::arg("remappings") = std::map<std::string, std::string>(),
          py::keep_alive<1, 3>()) // Keep state (arg 3) alive as long as self
                                  // (arg 1) is alive
      .def("set_name", &yasmin::StateMachine::set_name,
           "Set the name of the state machine", py::arg("name"))
      .def("get_name", &yasmin::StateMachine::get_name,
           "Get the name of the state machine")
      .def("set_start_state", &yasmin::StateMachine::set_start_state,
           "Set the initial state for the state machine", py::arg("state_name"))
      .def("get_start_state", &yasmin::StateMachine::get_start_state,
           "Get the name of the start state")
      .def(
          "get_states",
          [](yasmin::StateMachine &self) {
            // Return states in the old Python format for compatibility
            // with yasmin_viewer: {name: {"state": state, "transitions":
            // {...}}}
            py::dict result;
            const auto &states = self.get_states();
            const auto &transitions = self.get_transitions();

            for (const auto &[name, state] : states) {
              py::dict state_dict;
              state_dict["state"] = state;

              auto trans_it = transitions.find(name);
              if (trans_it != transitions.end()) {
                state_dict["transitions"] = trans_it->second;
              } else {
                state_dict["transitions"] = py::dict();
              }

              result[py::str(name)] = state_dict;
            }

            return result;
          },
          "Get all states with their transitions (Python compatibility format)")
      .def("_get_states_cpp", &yasmin::StateMachine::get_states,
           "Get all states in the state machine (C++ format)",
           py::return_value_policy::reference_internal)
      .def("get_transitions", &yasmin::StateMachine::get_transitions,
           "Get all transitions in the state machine",
           py::return_value_policy::reference_internal)
      .def("get_current_state", &yasmin::StateMachine::get_current_state,
           "Get the name of the current state being executed")
      .def(
          "add_start_cb",
          [](yasmin::StateMachine &self, py::function cb,
             std::vector<std::string> args) {
            // Wrap Python callback using utility function
            auto wrapped_cb =
                yasmin::pybind11_utils::wrap_blackboard_callback<void (*)(
                    std::shared_ptr<yasmin::blackboard::Blackboard>,
                    const std::string &, const std::vector<std::string> &)>(cb);
            self.add_start_cb(wrapped_cb, args);
          },
          "Add a callback to be called when the state machine starts",
          py::arg("cb"), py::arg("args") = std::vector<std::string>())
      .def(
          "add_transition_cb",
          [](yasmin::StateMachine &self, py::function cb,
             std::vector<std::string> args) {
            // Wrap Python callback using utility function
            auto wrapped_cb =
                yasmin::pybind11_utils::wrap_blackboard_callback<void (*)(
                    std::shared_ptr<yasmin::blackboard::Blackboard>,
                    const std::string &, const std::string &,
                    const std::string &, const std::vector<std::string> &)>(cb);
            self.add_transition_cb(wrapped_cb, args);
          },
          "Add a callback to be called during state transitions", py::arg("cb"),
          py::arg("args") = std::vector<std::string>())
      .def(
          "add_end_cb",
          [](yasmin::StateMachine &self, py::function cb,
             std::vector<std::string> args) {
            // Wrap Python callback using utility function
            auto wrapped_cb =
                yasmin::pybind11_utils::wrap_blackboard_callback<void (*)(
                    std::shared_ptr<yasmin::blackboard::Blackboard>,
                    const std::string &, const std::vector<std::string> &)>(cb);
            self.add_end_cb(wrapped_cb, args);
          },
          "Add a callback to be called when the state machine ends",
          py::arg("cb"), py::arg("args") = std::vector<std::string>())
      .def("validate", &yasmin::StateMachine::validate,
           "Validate the state machine configuration",
           py::arg("strict_mode") = false)
      .def("cancel_state", &yasmin::StateMachine::cancel_state,
           "Cancel the current state execution")
      .def("to_string", &yasmin::StateMachine::to_string,
           "Convert the state machine to a string representation")
      .def("__str__", &yasmin::StateMachine::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(sm_class),
                                            yasmin::StateMachine>(sm_class);
}
