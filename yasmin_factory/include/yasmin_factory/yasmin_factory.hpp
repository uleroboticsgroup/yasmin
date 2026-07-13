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

#ifndef YASMIN_FACTORY__YASMIN_FACTORY_HPP_
#define YASMIN_FACTORY__YASMIN_FACTORY_HPP_

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tinyxml2.h>

#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/join_state.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

namespace yasmin_factory {

/**
 * @class PythonStateHolder
 * @brief Minimal holder to keep Python state objects alive in C++.
 *
 * This class wraps a Python state object and delegates all operations
 * to the underlying PyState trampoline. Its only purpose is to maintain
 * the Python object's lifetime.
 */
class __attribute__((visibility("hidden"))) PythonStateHolder
    : public yasmin::State {
public:
  /**
   * @brief Constructs a holder with a Python state object.
   * @param cpp_state The C++ state pointer extracted from Python.
   * @param py_state The Python state object (kept alive).
   */
  PythonStateHolder(yasmin::State::SharedPtr cpp_state, py::object py_state);

  /**
   * @brief Synchronizes the wrapper parameter view with the Python state and
   *        delegates configuration to the underlying state.
   */
  void configure() override;

  /**
   * @brief Unwraps to the actual C++ state, allowing JoinState detection.
   * @return Pointer to the inner C++ state object.
   */
  yasmin::State *get_inner_state() override;

  /**
   * @brief Delegates execution to the underlying Python state.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome returned by the underlying Python state.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

  /**
   * @brief Delegates cancellation to the underlying Python state.
   */
  void cancel_state() override;

  /**
   * @brief Delegates string conversion to the underlying Python state.
   *
   * @return std::string The string representation from the underlying Python
   * state.
   */
  std::string to_string() const override;

private:
  yasmin::State::SharedPtr cpp_state_; ///< The C++ state pointer
  py::object py_state_;                ///< Python object (kept alive)
};

/**
 * @class YasminFactory
 * @brief Factory class to create state machines from XML files.
 *
 * This class provides methods to parse XML files and create yasmin state
 * machines, including states, concurrence, and nested state machines.
 */
class YasminFactory {
public:
  /**
   * @brief Constructs a YasminFactory instance.
   *
   * Initializes the pluginlib class loader for yasmin::State classes.
   */
  YasminFactory();

  /**
   * @brief Destructor.
   */
  ~YasminFactory();

  /**
   * @brief Creates a state from an XML element.
   *
   * @param state_elem Pointer to the XML element defining the state.
   * @return A shared pointer to the created State.
   * @throws std::runtime_error If the state type is unknown or required
   *         attributes are missing.
   */
  yasmin::State::SharedPtr create_state(tinyxml2::XMLElement *state_elem) const;

  /**
   * @brief Creates a concurrence from an XML element.
   *
   * @param conc_elem Pointer to the XML element defining the concurrence.
   * @return A shared pointer to the created Concurrence.
   * @throws std::runtime_error If required attributes are missing or the
   *         XML structure is invalid.
   */
  yasmin::Concurrence::SharedPtr
  create_concurrence(tinyxml2::XMLElement *conc_elem);

  /**
   * @brief Creates an orthogonal state from an XML element.
   *
   * @param orth_elem Pointer to the XML element defining the orthogonal state.
   * @return A shared pointer to the created OrthogonalState.
   * @throws std::runtime_error If the XML structure is invalid.
   */
  yasmin::OrthogonalState::SharedPtr
  create_orthogonal_state(tinyxml2::XMLElement *orth_elem);

  /**
   * @brief Creates a join state from an XML element.
   *
   * @param join_elem Pointer to the XML element defining the join state.
   * @return A shared pointer to the created JoinState.
   * @throws std::runtime_error If the XML structure is invalid.
   */
  yasmin::JoinState::SharedPtr
  create_join_state(tinyxml2::XMLElement *join_elem);

  /**
   * @brief Recursively creates a state machine from an XML element.
   *
   * @param root Pointer to the XML element defining the state machine.
   * @return A shared pointer to the created StateMachine.
   * @throws std::runtime_error If the XML structure is invalid.
   */
  yasmin::StateMachine::SharedPtr create_sm(tinyxml2::XMLElement *root);

  /**
   * @brief Creates a state machine from an XML file.
   *
   * @param xml_file Path to the XML file defining the state machine.
   * @return A shared pointer to the created StateMachine.
   * @throws std::runtime_error If the file cannot be loaded or the XML
   *         structure is invalid.
   */
  yasmin::StateMachine::SharedPtr
  create_sm_from_file(const std::string &xml_file);

  /**
   * @brief Explicitly cleanup the factory resources.
   *
   * This method should be called before the plugin loader is destroyed to
   * avoid class loader warnings.
   */
  void cleanup();

private:
  /// Pluginlib class loader for yasmin::State classes
  std::unique_ptr<pluginlib::ClassLoader<yasmin::State>> state_loader_;

  /// Python interpreter guard (initialized once)
  static std::unique_ptr<py::scoped_interpreter> py_interpreter_;

  /// Track if Python interpreter is initialized
  static bool py_initialized_;

  /// Path to the XML file being processed
  std::string xml_path_;

  /**
   * @brief Initializes the Python interpreter if not already initialized.
   */
  void initialize_python();

  /**
   * @brief Creates a Python state from module and class information.
   *
   * @param module_name The Python module name.
   * @param class_name The Python class name.
   * @return A shared pointer to the created Python state adapter.
   * @throws std::runtime_error If the Python state cannot be created.
   */
  yasmin::State::SharedPtr
  create_python_state(const std::string &module_name,
                      const std::string &class_name) const;

  /**
   * @brief Helper function to split a string by a delimiter.
   *
   * @param str The string to split.
   * @param delimiter The delimiter character.
   * @return A vector of strings.
   */
  std::vector<std::string> split_string(const std::string &str,
                                        char delimiter) const;

  /**
   * @brief Helper function to trim whitespace from a string.
   *
   * @param str The string to trim.
   * @return The trimmed string.
   */
  std::string trim(const std::string &str) const;

  /**
   * @brief Helper function to get a required attribute from an XML element.
   *
   * @param element Pointer to the XML element.
   * @param attr_name Name of the attribute.
   * @return The attribute value.
   * @throws std::runtime_error If the attribute is not found.
   */
  std::string get_required_attribute(tinyxml2::XMLElement *element,
                                     const std::string &attr_name) const;

  /**
   * @brief Helper function to get an optional attribute from an XML element.
   *
   * @param element Pointer to the XML element.
   * @param attr_name Name of the attribute.
   * @param default_value Default value if attribute is not found.
   * @return The attribute value or default value.
   */
  std::string
  get_optional_attribute(tinyxml2::XMLElement *element,
                         const std::string &attr_name,
                         const std::string &default_value = "") const;

  /**
   * @brief Adds blackboard key mappings from an XML element to a state.
   *
   * @param owner Pointer to the state to which the keys will be added.
   * @param parent XML element containing the blackboard key definitions.
   */
  void add_blackboard_keys(yasmin::State::SharedPtr owner,
                           tinyxml2::XMLElement *parent) const;

  /**
   * @brief Adds parameter mappings from an XML element to a state.
   *
   * @param owner Pointer to the state to which the parameters will be added.
   * @param parent XML element containing the parameter definitions.
   */
  void add_parameters(yasmin::State::SharedPtr owner,
                      tinyxml2::XMLElement *parent) const;

  /**
   * @brief Extracts parameter name mappings from an XML element.
   *
   * @param parent XML element containing the parameter mapping definitions.
   * @return yasmin::ParameterMappings A mapping of parameter names.
   */
  yasmin::ParameterMappings
  get_parameter_mappings(tinyxml2::XMLElement *parent) const;
};

} // namespace yasmin_factory

#endif // YASMIN_FACTORY__YASMIN_FACTORY_HPP_
