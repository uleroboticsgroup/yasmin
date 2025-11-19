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

#ifndef YASMIN_FACTORY__YASMIN_FACTORY_HPP
#define YASMIN_FACTORY__YASMIN_FACTORY_HPP

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tinyxml2.h>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

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
  PythonStateHolder(std::shared_ptr<yasmin::State> cpp_state,
                    py::object py_state);

  /**
   * @brief Delegates execution to the underlying Python state.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override;

  /**
   * @brief Delegates cancellation to the underlying Python state.
   */
  void cancel_state() override;

  /**
   * @brief Delegates string conversion to the underlying Python state.
   */
  std::string to_string() override;

private:
  std::shared_ptr<yasmin::State> cpp_state_; ///< The C++ state pointer
  py::object py_state_;                      ///< Python object (kept alive)
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
  std::shared_ptr<yasmin::State> create_state(tinyxml2::XMLElement *state_elem);

  /**
   * @brief Creates a concurrence from an XML element.
   *
   * @param conc_elem Pointer to the XML element defining the concurrence.
   * @return A shared pointer to the created Concurrence.
   * @throws std::runtime_error If required attributes are missing or the
   *         XML structure is invalid.
   */
  std::shared_ptr<yasmin::Concurrence>
  create_concurrence(tinyxml2::XMLElement *conc_elem);

  /**
   * @brief Recursively creates a state machine from an XML element.
   *
   * @param root Pointer to the XML element defining the state machine.
   * @return A shared pointer to the created StateMachine.
   * @throws std::runtime_error If the XML structure is invalid.
   */
  std::shared_ptr<yasmin::StateMachine> create_sm(tinyxml2::XMLElement *root);

  /**
   * @brief Creates a state machine from an XML file.
   *
   * @param xml_file Path to the XML file defining the state machine.
   * @return A shared pointer to the created StateMachine.
   * @throws std::runtime_error If the file cannot be loaded or the XML
   *         structure is invalid.
   */
  std::shared_ptr<yasmin::StateMachine>
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
  std::shared_ptr<yasmin::State>
  create_python_state(const std::string &module_name,
                      const std::string &class_name);

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
};

} // namespace yasmin_factory

#endif // YASMIN_FACTORY__YASMIN_FACTORY_HPP
