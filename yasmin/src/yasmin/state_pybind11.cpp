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

#include "yasmin/blackboard_key_info_py.hpp"
#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/pybind11_utils.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

// Declare that BlackboardPyWrapper is defined in another module
// This allows us to use it without re-registering it
PYBIND11_MAKE_OPAQUE(yasmin::BlackboardPyWrapper);

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
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    // Acquire GIL before calling Python code
    py::gil_scoped_acquire acquire;

    // Wrap the C++ Blackboard in BlackboardPyWrapper for Python
    yasmin::BlackboardPyWrapper wrapper(blackboard);

#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
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
   * @brief Override configure() to call Python implementation if available.
   * The GIL must be acquired before calling into Python.
   */
  void configure() override {
    py::gil_scoped_acquire acquire;

#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
    PYBIND11_OVERRIDE(void,     // Return type
                      State,    // Parent class
                      configure // Method name (no arguments)
    );
#else
    PYBIND11_OVERLOAD(void,     // Return type
                      State,    // Parent class
                      configure // Method name (no arguments)
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

#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
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
  std::string to_string() const override {
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
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
  py::module_::import("yasmin.callback_signal");
#else
  py::module::import("yasmin.callback_signal");
#endif
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
  py::module_ blackboard_module = py::module_::import("yasmin.blackboard");
#else
  py::module blackboard_module = py::module::import("yasmin.blackboard");
#endif

  // Export State class with trampoline
  py::class_<yasmin::State, yasmin::PyState, yasmin::State::SharedPtr>
      state_class(m, "State");

  state_class.def(py::init<yasmin::Outcomes>(), py::arg("outcomes"))
      .def(py::init([](const std::vector<std::string> &outcomes) {
             return new yasmin::PyState(
                 yasmin::Outcomes(outcomes.begin(), outcomes.end()));
           }),
           py::arg("outcomes"))
      // Status methods
      .def("is_idle", &yasmin::State::is_idle, "Checks if the state is idle")
      .def("is_running", &yasmin::State::is_running,
           "Checks if the state is currently running")
      .def("is_canceled", &yasmin::State::is_canceled,
           "Checks if the state has been canceled")
      .def("is_completed", &yasmin::State::is_completed,
           "Checks if the state has completed execution")
      // Execute, configure, and cancel methods
      .def("execute", &yasmin::State::execute,
           "Execute the state's specific logic (override in subclass)",
           py::arg("blackboard"))
      .def("configure", &yasmin::State::configure,
           "Configure the state before execution")
      .def("cancel_state", &yasmin::State::cancel_state,
           "Cancel the current state execution")
      // Get outcomes method
      .def("get_outcomes", &yasmin::State::get_outcomes,
           "Get the set of possible outcomes for this state")
      // Metadata methods
      .def("set_description", &yasmin::State::set_description,
           "Set the description for this state", py::arg("description"))
      .def("get_description", &yasmin::State::get_description,
           "Get the description of this state")
      .def("set_outcome_description", &yasmin::State::set_outcome_description,
           "Set the description for an outcome", py::arg("outcome"),
           py::arg("description"))
      .def("get_outcome_description", &yasmin::State::get_outcome_description,
           "Get the description of an outcome", py::arg("outcome"))
      .def("get_outcome_descriptions", &yasmin::State::get_outcome_descriptions,
           "Get all outcome descriptions")

      .def(
          "add_input_key",
          [](yasmin::State &state, const std::string &key_name) {
            state.add_input_key(key_name);
          },
          "Add an input key with name only", py::arg("key_name"))

      .def(
          "add_input_key",
          [](yasmin::State &state, const std::string &key_name,
             const std::string &description) {
            yasmin::BlackboardKeyInfo info(key_name);
            info.description = description;
            state.add_input_key(info);
          },
          "Add an input key with name and description", py::arg("key_name"),
          py::arg("description"))

      .def(
          "add_input_key",
          [](yasmin::State &state, const std::string &key_name,
             const std::string &description, py::object default_value) {
            yasmin::BlackboardKeyInfo info =
                yasmin::BlackboardKeyInfoPy::from_pyobject(key_name,
                                                           default_value);
            info.description = description;
            state.add_input_key(info);
          },
          "Add an input key with a default value of any type and description",
          py::arg("key_name"), py::arg("description"), py::arg("default_value"))

      .def(
          "add_output_key",
          [](yasmin::State &state, const std::string &key_name) {
            state.add_output_key(key_name);
          },
          "Add an output key with name only", py::arg("key_name"))

      .def(
          "add_output_key",
          [](yasmin::State &state, const std::string &key_name,
             const std::string &description) {
            yasmin::BlackboardKeyInfo info(key_name);
            info.description = description;
            state.add_output_key(info);
          },
          "Add an output key with name and description", py::arg("key_name"),
          py::arg("description"))

      .def(
          "declare_parameter",
          [](yasmin::State &state, const std::string &parameter_name) {
            state.declare_parameter(parameter_name);
          },
          "Declare a parameter with name only", py::arg("parameter_name"))

      .def(
          "declare_parameter",
          [](yasmin::State &state, const std::string &parameter_name,
             const std::string &description) {
            yasmin::BlackboardKeyInfo info(parameter_name);
            info.description = description;
            state.declare_parameter(info);
          },
          "Declare a parameter with name and description",
          py::arg("parameter_name"), py::arg("description"))

      .def(
          "declare_parameter",
          [](yasmin::State &state, const std::string &parameter_name,
             const std::string &description, py::object default_value) {
            yasmin::BlackboardKeyInfo info =
                yasmin::BlackboardKeyInfoPy::from_pyobject(parameter_name,
                                                           default_value);
            info.description = description;
            state.declare_parameter(info);
          },
          "Declare a parameter with a default value of any type and "
          "description",
          py::arg("parameter_name"), py::arg("description"),
          py::arg("default_value"))

      .def("has_parameter", &yasmin::State::has_parameter,
           "Check whether a parameter exists in the local parameter storage",
           py::arg("parameter_name"))

      .def(
          "get_parameter",
          [](const yasmin::State &state, const std::string &parameter_name) {
            yasmin::BlackboardPyWrapper wrapper(
                state.get_parameters_blackboard());
            return wrapper.get(parameter_name);
          },
          "Get a parameter from the local parameter storage",
          py::arg("parameter_name"))

      .def(
          "set_parameter",
          [](yasmin::State &state, const std::string &parameter_name,
             py::object value) {
            yasmin::BlackboardPyWrapper wrapper(
                state.get_parameters_blackboard());
            wrapper.set(parameter_name, value);
          },
          "Set a parameter in the local parameter storage",
          py::arg("parameter_name"), py::arg("value"))

      .def(
          "get_parameters",
          [](const yasmin::State &state) {
            const auto &keys = state.get_parameters();
            py::list result;
            for (const auto &key : keys) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              result.append(key_dict);
            }
            return result;
          },
          "Get the parameter metadata")

      .def(
          "get_input_keys",
          [](const yasmin::State &state) {
            const auto &keys = state.get_input_keys();
            py::list result;
            for (const auto &key : keys) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              result.append(key_dict);
            }
            return result;
          },
          "Get the input keys metadata")

      .def(
          "get_output_keys",
          [](const yasmin::State &state) {
            const auto &keys = state.get_output_keys();
            py::list result;
            for (const auto &key : keys) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              result.append(key_dict);
            }
            return result;
          },
          "Get the output keys metadata")

      .def(
          "get_metadata",
          [](const yasmin::State &state) {
            const auto &metadata = state.get_metadata();
            py::dict result;
            result["description"] = metadata.description;

            py::list input_keys;
            for (const auto &key : metadata.input_keys) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              input_keys.append(key_dict);
            }

            result["input_keys"] = input_keys;

            py::list output_keys;
            for (const auto &key : metadata.output_keys) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              output_keys.append(key_dict);
            }

            result["output_keys"] = output_keys;

            py::list parameters;
            for (const auto &key : metadata.parameters) {
              py::dict key_dict;
              key_dict["name"] = key.name;
              key_dict["description"] = key.description;
              key_dict["has_default"] = key.has_default;
              if (key.has_default) {
                key_dict["default_value_type"] = key.default_value_type;
                key_dict["default_value"] =
                    yasmin::BlackboardKeyInfoPy::get_py_default_value(key);
              }
              parameters.append(key_dict);
            }

            result["parameters"] = parameters;

            py::dict outcome_descriptions;
            for (const auto &[outcome, description] :
                 metadata.outcome_descriptions) {
              outcome_descriptions[py::str(outcome)] = py::str(description);
            }

            result["outcome_descriptions"] = outcome_descriptions;
            return result;
          },
          "Get the complete state metadata")

      // String representation
      .def("to_string", &yasmin::State::to_string,
           "Convert the state to a string representation")
      .def("__str__", &yasmin::State::to_string);

  // Add the __call__ operator using the utility function
  yasmin::pybind11_utils::add_call_operator<decltype(state_class),
                                            yasmin::State>(state_class);
}
