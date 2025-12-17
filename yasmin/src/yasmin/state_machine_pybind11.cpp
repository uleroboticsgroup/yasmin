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

#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/pybind11_utils.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::BlackboardPyWrapper);

PYBIND11_MODULE(state_machine, m) {
  m.doc() = "Python bindings for yasmin::StateMachine";

  // Export StateMachine class - inherits from State
  py::class_<yasmin::StateMachine,
             yasmin::State, // Inherit from State
             yasmin::StateMachine::SharedPtr /* Shared pointer */>
      sm_class(m, "StateMachine");

  sm_class.def(py::init<yasmin::Outcomes>(), py::arg("outcomes"))
      .def(py::init([](const std::vector<std::string> &outcomes,
                       bool handle_sigint) {
             return new yasmin::StateMachine(
                 yasmin::Outcomes(outcomes.begin(), outcomes.end()),
                 handle_sigint);
           }),
           py::arg("outcomes"), py::arg("handle_sigint") = false)
      .def(py::init<const std::string &, const yasmin::Outcomes &, bool>(),
           py::arg("name"), py::arg("outcomes"),
           py::arg("handle_sigint") = false)
      .def(py::init([](const std::string &name,
                       const std::vector<std::string> &outcomes,
                       bool handle_sigint) {
             return new yasmin::StateMachine(
                 name, std::set<std::string>(outcomes.begin(), outcomes.end()),
                 handle_sigint);
           }),
           py::arg("name"), py::arg("outcomes"),
           py::arg("handle_sigint") = false)
      // Add state method with keep_alive to manage object lifetime
      .def(
          "add_state",
          [](yasmin::StateMachine &self, const std::string &name,
             yasmin::State::SharedPtr state,
             const yasmin::Transitions &transitions,
             const yasmin::Remappings &remappings) {
            // Ensure the Python object is kept alive
            py::object py_state = py::cast(state);
            self.add_state(name, state, transitions, remappings);
          },
          "Add a state to the state machine", py::arg("name"), py::arg("state"),
          py::arg("transitions") = yasmin::Transitions(),
          py::arg("remappings") = yasmin::Remappings(),
          py::keep_alive<1, 3>()) // Keep state (arg 3) alive as long as self
                                  // (arg 1) is alive
      // Setters and getters for name and start state
      .def("set_name", &yasmin::StateMachine::set_name,
           "Set the name of the state machine", py::arg("name"))
      .def("get_name", &yasmin::StateMachine::get_name,
           "Get the name of the state machine")
      .def("set_start_state", &yasmin::StateMachine::set_start_state,
           "Set the initial state for the state machine", py::arg("state_name"))
      .def("get_start_state", &yasmin::StateMachine::get_start_state,
           "Get the name of the start state")
      // Methods to get states, transitions, current state
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
      // Callback registration methods
      .def(
          "add_start_cb",
          [](yasmin::StateMachine &self, py::function cb) {
            // Wrap Python callback using utility function
            auto wrapped_cb = yasmin::pybind11_utils::wrap_blackboard_callback<
                yasmin::StateMachine::StartCallbackType>(cb);
            self.add_start_cb(wrapped_cb);
          },
          "Add a callback to be called when the state machine starts",
          py::arg("cb"))
      .def(
          "add_transition_cb",
          [](yasmin::StateMachine &self, py::function cb) {
            // Wrap Python callback using utility function
            auto wrapped_cb = yasmin::pybind11_utils::wrap_blackboard_callback<
                yasmin::StateMachine::TransitionCallbackType>(cb);
            self.add_transition_cb(wrapped_cb);
          },
          "Add a callback to be called during state transitions", py::arg("cb"))
      .def(
          "add_end_cb",
          [](yasmin::StateMachine &self, py::function cb) {
            // Wrap Python callback using utility function
            auto wrapped_cb = yasmin::pybind11_utils::wrap_blackboard_callback<
                yasmin::StateMachine::EndCallbackType>(cb);
            self.add_end_cb(wrapped_cb);
          },
          "Add a callback to be called when the state machine ends",
          py::arg("cb"))
      // Validation and cancellation methods
      .def("validate", &yasmin::StateMachine::validate,
           "Validate the state machine configuration",
           py::arg("strict_mode") = false)
      .def("cancel_state", &yasmin::StateMachine::cancel_state,
           "Cancel the current state execution")
      .def("set_sigint_handler", &yasmin::StateMachine::set_sigint_handler,
           "Set whether the state machine should handle SIGINT for cancel",
           py::arg("handle") = true)
      // String representation
      .def("to_string", &yasmin::StateMachine::to_string,
           "Convert the state machine to a string representation")
      .def("__str__", &yasmin::StateMachine::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(sm_class),
                                            yasmin::StateMachine>(sm_class);
}
