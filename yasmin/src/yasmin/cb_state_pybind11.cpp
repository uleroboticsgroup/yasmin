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
#include "yasmin/cb_state.hpp"
#include "yasmin/pybind11_utils.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::blackboard::BlackboardPyWrapper);

PYBIND11_MODULE(cb_state, m) {
  m.doc() = "Python bindings for yasmin::CbState";

  // Export CbState class - inherits from State
  py::class_<yasmin::CbState, yasmin::State, std::shared_ptr<yasmin::CbState>>
      cb_state_class(m, "CbState");

  cb_state_class
      .def(py::init([](const std::set<std::string> &outcomes,
                       py::function callback) {
             return new yasmin::CbState(
                 outcomes,
                 yasmin::pybind11_utils::wrap_blackboard_callback_with_return<
                     std::string>(callback));
           }),
           py::arg("outcomes"), py::arg("callback"),
           "Constructs a CbState object with outcomes and a callback function")
      .def(py::init([](const std::vector<std::string> &outcomes,
                       py::function callback) {
             std::set<std::string> outcomes_set(outcomes.begin(),
                                                outcomes.end());
             return new yasmin::CbState(
                 outcomes_set,
                 yasmin::pybind11_utils::wrap_blackboard_callback_with_return<
                     std::string>(callback));
           }),
           py::arg("outcomes"), py::arg("callback"),
           "Constructs a CbState object with outcomes (list) and a callback "
           "function")
      .def("execute", &yasmin::CbState::execute,
           "Execute the callback function with the provided blackboard",
           py::arg("blackboard"))
      .def("to_string", &yasmin::CbState::to_string,
           "Convert the CbState to a string representation")
      .def("__str__", &yasmin::CbState::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(cb_state_class),
                                            yasmin::CbState>(cb_state_class);
}
