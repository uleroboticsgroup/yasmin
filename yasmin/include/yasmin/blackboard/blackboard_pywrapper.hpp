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

#ifndef YASMIN__BLACKBOARD__BLACKBOARD_PYWRAPPER_HPP
#define YASMIN__BLACKBOARD__BLACKBOARD_PYWRAPPER_HPP

#include <list>
#include <map>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_pyobject.hpp"

namespace py = pybind11;

namespace yasmin {
namespace blackboard {

/**
 * @class BlackboardPyWrapper
 * @brief A wrapper around the C++ Blackboard that stores Python objects and
 * native types.
 *
 * This wrapper provides a Python-friendly interface to the C++ Blackboard,
 * allowing storage and retrieval of both arbitrary Python objects and native
 * C++ types (int, float, string, bool, etc.) while maintaining thread safety
 * and proper memory management.
 *
 * Supported types:
 * - Python int -> C++ int64_t
 * - Python float -> C++ double
 * - Python str -> C++ std::string
 * - Python bool -> C++ bool
 * - Python list -> C++ std::vector<py::object>
 * - Python tuple -> C++ std::vector<py::object>
 * - Python dict -> C++ std::map<std::string, py::object>
 * - Other Python objects -> Stored as PyObject*
 */
class BlackboardPyWrapper {
private:
  std::shared_ptr<Blackboard> blackboard;

public:
  BlackboardPyWrapper() : blackboard(std::make_shared<Blackboard>()) {}

  /**
   * @brief Construct from an existing C++ Blackboard (move constructor)
   */
  BlackboardPyWrapper(Blackboard &&other)
      : blackboard(std::make_shared<Blackboard>(std::move(other))) {}

  /**
   * @brief Construct by wrapping a shared_ptr to a C++ Blackboard
   */
  explicit BlackboardPyWrapper(std::shared_ptr<Blackboard> bb_ptr)
      : blackboard(bb_ptr) {}

  /**
   * @brief Set a Python object in the blackboard.
   *
   * This method automatically detects the type of the Python object and stores
   * it in the most appropriate C++ representation:
   * - Python int -> C++ int64_t
   * - Python float -> C++ double
   * - Python str -> C++ std::string
   * - Python bool -> C++ bool
   * - Python list -> C++ std::vector<py::object>
   * - Other Python objects -> Stored as PyObject*
   *
   * @param key The key to associate with the value.
   * @param value The Python object to store.
   */
  void set(const std::string &key, py::object value) {
    // Check for None first
    if (value.is_none()) {
      this->blackboard->set<BlackboardValuePyObject *>(
          key, new BlackboardValuePyObject(value.ptr()));
      return;
    }

    // Check for bool (must be before int, as bool is a subtype of int in
    // Python)
    if (py::isinstance<py::bool_>(value)) {
      bool cpp_value = value.cast<bool>();
      this->blackboard->set<bool>(key, cpp_value);
      return;
    }

    // Check for int
    if (py::isinstance<py::int_>(value)) {
      int64_t cpp_value = value.cast<int64_t>();
      this->blackboard->set<int64_t>(key, cpp_value);
      return;
    }

    // Check for float
    if (py::isinstance<py::float_>(value)) {
      double cpp_value = value.cast<double>();
      this->blackboard->set<double>(key, cpp_value);
      return;
    }

    // Check for str
    if (py::isinstance<py::str>(value)) {
      std::string cpp_value = value.cast<std::string>();
      this->blackboard->set<std::string>(key, cpp_value);
      return;
    }

    // Check for list
    if (py::isinstance<py::list>(value)) {
      py::list py_list = value.cast<py::list>();
      std::vector<py::object> cpp_vector;
      for (auto item : py_list) {
        cpp_vector.push_back(py::reinterpret_borrow<py::object>(item));
      }
      this->blackboard->set<std::vector<py::object>>(key, cpp_vector);
      return;
    }

    // Check for tuple
    if (py::isinstance<py::tuple>(value)) {
      py::tuple py_tuple = value.cast<py::tuple>();
      std::vector<py::object> cpp_vector;
      for (auto item : py_tuple) {
        cpp_vector.push_back(py::reinterpret_borrow<py::object>(item));
      }
      this->blackboard->set<std::vector<py::object>>(key, cpp_vector);
      return;
    }

    // Check for dict
    if (py::isinstance<py::dict>(value)) {
      py::dict py_dict = value.cast<py::dict>();
      std::map<std::string, py::object> cpp_map;
      for (auto item : py_dict) {
        std::string map_key = py::str(item.first).cast<std::string>();
        cpp_map[map_key] = py::reinterpret_borrow<py::object>(item.second);
      }
      this->blackboard->set<std::map<std::string, py::object>>(key, cpp_map);
      return;
    }

    // Default: store as PyObject* for other Python objects
    PyObject *py_obj = value.ptr();

    if (this->blackboard->contains(key)) {
      BlackboardValuePyObject *existing =
          this->blackboard->get<BlackboardValuePyObject *>(key);
      existing->set(py_obj);
    } else {
      this->blackboard->set<BlackboardValuePyObject *>(
          key, new BlackboardValuePyObject(py_obj));
    }
  }

  /**
   * @brief Get a Python object from the blackboard.
   *
   * This method retrieves values from the blackboard and converts them back to
   * Python objects. It automatically handles:
   * - C++ int64_t -> Python int
   * - C++ double -> Python float
   * - C++ std::string -> Python str
   * - C++ bool -> Python bool
   * - C++ std::vector<py::object> -> Python list
   * - C++ std::map<std::string, py::object> -> Python dict
   * - PyObject* -> Python object
   *
   * @param key The key associated with the value.
   * @return The Python object.
   * @throws std::runtime_error if the key does not exist.
   */
  py::object get(const std::string &key) {
    if (!this->blackboard->contains(key)) {
      throw std::runtime_error("Element '" + key +
                               "' does not exist in the blackboard");
    }

    // Try string first (most common and avoids false positives with bool)
    try {
      std::string value = this->blackboard->get<std::string>(key);
      return py::cast(value);
    } catch (...) {
      // Not a string, continue
    }

    // Try vector
    try {
      std::vector<py::object> value =
          this->blackboard->get<std::vector<py::object>>(key);
      py::list py_list;
      for (const auto &item : value) {
        py_list.append(item);
      }
      return py_list;
    } catch (...) {
      // Not a vector, continue
    }

    // Try map
    try {
      std::map<std::string, py::object> value =
          this->blackboard->get<std::map<std::string, py::object>>(key);
      py::dict py_dict;
      for (const auto &item : value) {
        py_dict[py::str(item.first)] = item.second;
      }
      return py_dict;
    } catch (...) {
      // Not a map, continue
    }

    // Try double (before int to preserve floating point precision)
    try {
      double value = this->blackboard->get<double>(key);
      return py::cast(value);
    } catch (...) {
      // Not a double, continue
    }

    // Try int64_t
    try {
      int64_t value = this->blackboard->get<int64_t>(key);
      return py::cast(value);
    } catch (...) {
      // Not an int, continue
    }

    // Try bool (checked after numeric types to avoid false conversions)
    try {
      bool value = this->blackboard->get<bool>(key);
      return py::cast(value);
    } catch (...) {
      // Not a bool, continue
    }

    // Try PyObject* (fallback)
    try {
      BlackboardValuePyObject *wrapper =
          this->blackboard->get<BlackboardValuePyObject *>(key);
      PyObject *py_obj = wrapper->get();
      return py::reinterpret_borrow<py::object>(py_obj);
    } catch (...) {
      // If we get here, something went wrong
      throw std::runtime_error("Unable to retrieve value for key '" + key +
                               "' from blackboard");
    }
  }

  /**
   * @brief Remove a value from the blackboard.
   * @param key The key associated with the value to remove.
   */
  void remove(const std::string &key) { this->blackboard->remove(key); }

  /**
   * @brief Check if a key exists in the blackboard.
   * @param key The key to check.
   * @return True if the key exists, false otherwise.
   */
  bool contains(const std::string &key) {
    return this->blackboard->contains(key);
  }

  /**
   * @brief Get the number of key-value pairs in the blackboard.
   * @return The size of the blackboard.
   */
  int size() { return this->blackboard->size(); }

  /**
   * @brief Convert the contents of the blackboard to a string.
   * @return A string representation of the blackboard.
   */
  std::string to_string() { return this->blackboard->to_string(); }

  /**
   * @brief Set the remapping of the blackboard.
   * @param remapping The remapping to set.
   */
  void set_remapping(const std::map<std::string, std::string> &remapping) {
    this->blackboard->set_remapping(remapping);
  }

  /**
   * @brief Get the remapping of the blackboard.
   * @return The remapping of the blackboard.
   */
  std::map<std::string, std::string> get_remapping() {
    return this->blackboard->get_remapping();
  }

  /**
   * @brief Get a shared pointer to the underlying C++ Blackboard
   * @return Shared pointer to the C++ Blackboard
   */
  std::shared_ptr<Blackboard> get_cpp_blackboard() { return this->blackboard; }
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN__BLACKBOARD__BLACKBOARD_PYWRAPPER_HPP
