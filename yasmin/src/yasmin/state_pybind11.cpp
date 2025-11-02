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
#include "yasmin/pybind11_utils.hpp"
#include "yasmin/state.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
// This allows us to use it without re-registering it
PYBIND11_MAKE_OPAQUE(yasmin::blackboard::BlackboardPyWrapper);

namespace yasmin {

/**
 * @class PyState
 * @brief Trampoline class to enable Python classes to inherit from C++ State.
 *
 * This class allows Python code to override the virtual methods of the State
 * class, particularly the execute() method, while maintaining C++ type safety
 * and performance.
 */
class PyState : public State {
public:
  using State::State; // Inherit constructors

  /**
   * @brief Override execute() to call Python implementation.
   * We wrap the C++ Blackboard in BlackboardPyWrapper before passing to Python.
   * The GIL must be acquired before calling into Python.
   */
  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override {
    // Acquire GIL before calling Python code
    py::gil_scoped_acquire acquire;

    // Wrap the C++ Blackboard in BlackboardPyWrapper for Python
    blackboard::BlackboardPyWrapper wrapper(blackboard);

#if __has_include("rclcpp/version.h")
    PYBIND11_OVERRIDE_PURE(std::string, // Return type
                           State,       // Parent class
                           execute,     // Method name
                           wrapper      // Wrapped blackboard
    );
#else
    PYBIND11_OVERLOAD_PURE(std::string, // Return type
                           State,       // Parent class
                           execute,     // Method name
                           wrapper      // Wrapped blackboard
    );
#endif
  }

  /**
   * @brief Override cancel_state() to call Python implementation if available.
   * The GIL must be acquired before calling into Python.
   */
  void cancel_state() override {
    // Acquire GIL before calling Python code
    py::gil_scoped_acquire acquire;

#if __has_include("rclcpp/version.h")
    PYBIND11_OVERRIDE(void,        // Return type
                      State,       // Parent class
                      cancel_state // Method name (no arguments)
    );
#else
    PYBIND11_OVERLOAD(void,        // Return type
                      State,       // Parent class
                      cancel_state // Method name (no arguments)
    );
#endif
  }

  /**
   * @brief Override to_string().
   * The GIL must be acquired before calling into Python.
   */
  std::string to_string() override {
    // Acquire GIL before calling Python code
    py::gil_scoped_acquire acquire;

    // Return Python class name as string
    return py::str(py::cast(this).attr("__class__").attr("__name__"));
  }
};

} // namespace yasmin

PYBIND11_MODULE(state, m) {
  m.doc() = "Python bindings for yasmin::State";

  // Import BlackboardPyWrapper from blackboard module
  // This allows us to use it without re-registering the type
#if __has_include("rclcpp/version.h")
  py::module_ blackboard_module = py::module_::import("yasmin.blackboard");
#else
  py::module blackboard_module = py::module::import("yasmin.blackboard");
#endif

  // Export StateStatus enum
  py::enum_<yasmin::StateStatus>(m, "StateStatus")
      .value("IDLE", yasmin::StateStatus::IDLE)
      .value("RUNNING", yasmin::StateStatus::RUNNING)
      .value("CANCELED", yasmin::StateStatus::CANCELED)
      .value("COMPLETED", yasmin::StateStatus::COMPLETED)
      .export_values();

  // Export State class with trampoline
  py::class_<yasmin::State, yasmin::PyState, std::shared_ptr<yasmin::State>>
      state_class(m, "State");

  state_class.def(py::init<std::set<std::string>>(), py::arg("outcomes"))
      .def(py::init([](const std::vector<std::string> &outcomes) {
             return new yasmin::PyState(
                 std::set<std::string>(outcomes.begin(), outcomes.end()));
           }),
           py::arg("outcomes"))
      .def("get_status", &yasmin::State::get_status,
           "Gets the current status of the state")
      .def("is_idle", &yasmin::State::is_idle, "Checks if the state is idle")
      .def("is_running", &yasmin::State::is_running,
           "Checks if the state is currently running")
      .def("is_canceled", &yasmin::State::is_canceled,
           "Checks if the state has been canceled")
      .def("is_completed", &yasmin::State::is_completed,
           "Checks if the state has completed execution")
      .def("execute", &yasmin::State::execute,
           "Execute the state's specific logic (override in subclass)",
           py::arg("blackboard"))
      .def("cancel_state", &yasmin::State::cancel_state,
           "Cancel the current state execution")
      .def("get_outcomes", &yasmin::State::get_outcomes,
           "Get the set of possible outcomes for this state")
      .def("to_string", &yasmin::State::to_string,
           "Convert the state to a string representation")
      .def("__str__", &yasmin::State::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(state_class),
                                            yasmin::State>(state_class);
}
