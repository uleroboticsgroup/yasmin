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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin/pybind11_utils.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
PYBIND11_MAKE_OPAQUE(yasmin::BlackboardPyWrapper);

PYBIND11_MODULE(cb_state, m) {
  m.doc() = "Python bindings for yasmin::CbState";

  // Export CbState class - inherits from State
  py::class_<yasmin::CbState, yasmin::State, yasmin::CbState::SharedPtr>
      cb_state_class(m, "CbState");

  cb_state_class
      .def(
          py::init([](const yasmin::Outcomes &outcomes, py::function callback) {
            return new yasmin::CbState(
                outcomes,
                yasmin::pybind11_utils::wrap_blackboard_callback_with_return<
                    std::string>(callback));
          }),
          py::arg("outcomes"), py::arg("callback"),
          "Constructs a CbState object with outcomes and a callback function")
      .def(py::init([](const std::vector<std::string> &outcomes,
                       py::function callback) {
             yasmin::Outcomes outcomes_set(outcomes.begin(), outcomes.end());
             return new yasmin::CbState(
                 outcomes_set,
                 yasmin::pybind11_utils::wrap_blackboard_callback_with_return<
                     std::string>(callback));
           }),
           py::arg("outcomes"), py::arg("callback"),
           "Constructs a CbState object with outcomes (list) and a callback "
           "function")
      // Execute method
      .def("execute", &yasmin::CbState::execute,
           "Execute the callback function with the provided blackboard",
           py::arg("blackboard"))
      // String representation
      .def("to_string", &yasmin::CbState::to_string,
           "Convert the CbState to a string representation")
      .def("__str__", &yasmin::CbState::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(cb_state_class),
                                            yasmin::CbState>(cb_state_class);
}
