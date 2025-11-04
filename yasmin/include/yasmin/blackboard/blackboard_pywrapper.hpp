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
#include <tuple>
#include <type_traits>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_pyobject.hpp"

namespace py = pybind11;

namespace yasmin {
namespace blackboard {

// Forward declaration
class BlackboardPyWrapper;

namespace detail {

/**
 * @brief Type traits for Python to C++ type mapping
 */
template <typename T> struct PythonTypeChecker {
  static bool check(py::handle) { return false; }
  static T extract(py::handle) { throw std::runtime_error("Invalid type"); }
};

// Specialization for bool (must come before int)
template <> struct PythonTypeChecker<bool> {
  static bool check(py::handle obj) { return PyBool_Check(obj.ptr()); }
  static bool extract(py::handle obj) { return obj.cast<bool>(); }
};

// Specialization for int64_t
template <> struct PythonTypeChecker<int64_t> {
  static bool check(py::handle obj) {
    return PyLong_Check(obj.ptr()) && !PyBool_Check(obj.ptr());
  }
  static int64_t extract(py::handle obj) { return obj.cast<int64_t>(); }
};

// Specialization for double
template <> struct PythonTypeChecker<double> {
  static bool check(py::handle obj) { return PyFloat_Check(obj.ptr()); }
  static double extract(py::handle obj) { return obj.cast<double>(); }
};

// Specialization for std::string
template <> struct PythonTypeChecker<std::string> {
  static bool check(py::handle obj) { return PyUnicode_Check(obj.ptr()); }
  static std::string extract(py::handle obj) { return obj.cast<std::string>(); }
};

// Specialization for std::vector<py::object>
template <> struct PythonTypeChecker<std::vector<py::object>> {
  static bool check(py::handle obj) {
    return PyList_Check(obj.ptr()) || PyTuple_Check(obj.ptr());
  }
  static std::vector<py::object> extract(py::handle obj) {
    std::vector<py::object> result;
    py::sequence seq = py::reinterpret_borrow<py::sequence>(obj);
    for (auto item : seq) {
      result.push_back(py::reinterpret_borrow<py::object>(item));
    }
    return result;
  }
};

// Specialization for std::map<std::string, py::object>
template <> struct PythonTypeChecker<std::map<std::string, py::object>> {
  static bool check(py::handle obj) { return PyDict_Check(obj.ptr()); }
  static std::map<std::string, py::object> extract(py::handle obj) {
    std::map<std::string, py::object> result;
    py::dict dict = py::reinterpret_borrow<py::dict>(obj);
    for (auto item : dict) {
      std::string key = py::str(item.first).cast<std::string>();
      result[key] = py::reinterpret_borrow<py::object>(item.second);
    }
    return result;
  }
};

/**
 * @brief Attempts to set a value in the blackboard using template recursion
 */
template <typename... Types> struct TypeSetter;

template <> struct TypeSetter<> {
  static bool try_set(Blackboard *, const std::string &, py::object) {
    return false; // No more types to try
  }
};

template <typename T, typename... Rest> struct TypeSetter<T, Rest...> {
  static bool try_set(Blackboard *bb, const std::string &key,
                      py::object value) {
    if (PythonTypeChecker<T>::check(value)) {
      bb->set<T>(key, PythonTypeChecker<T>::extract(value));
      return true;
    }
    return TypeSetter<Rest...>::try_set(bb, key, value);
  }
};

/**
 * @brief Attempts to get a value from the blackboard using template recursion
 */
template <typename... Types> struct TypeGetter;

template <> struct TypeGetter<> {
  static std::pair<bool, py::object> try_get(Blackboard *,
                                             const std::string &) {
    return {false, py::none()}; // No more types to try
  }
};

template <typename T, typename... Rest> struct TypeGetter<T, Rest...> {
  static std::pair<bool, py::object> try_get(Blackboard *bb,
                                             const std::string &key) {
    try {
      T value = bb->get<T>(key);
      return {true, convert_to_python(value)};
    } catch (...) {
      return TypeGetter<Rest...>::try_get(bb, key);
    }
  }

private:
  // Convert C++ type to Python object
  static py::object convert_to_python(const std::string &value) {
    return py::cast(value);
  }

  static py::object convert_to_python(bool value) { return py::cast(value); }

  static py::object convert_to_python(int64_t value) { return py::cast(value); }

  static py::object convert_to_python(double value) { return py::cast(value); }

  static py::object convert_to_python(const std::vector<py::object> &value) {
    py::list result;
    for (const auto &item : value) {
      result.append(item);
    }
    return result;
  }

  static py::object
  convert_to_python(const std::map<std::string, py::object> &value) {
    py::dict result;
    for (const auto &item : value) {
      result[py::str(item.first)] = item.second;
    }
    return result;
  }
};

// Define the type list for supported types
// Order matters: bool before int64_t, string first for performance
using SupportedTypes =
    std::tuple<std::string, bool, int64_t, double, std::vector<py::object>,
               std::map<std::string, py::object>>;

// Helper to convert tuple to type list for TypeSetter and TypeGetter
template <typename Tuple> struct TupleToTypes;

template <typename... Types> struct TupleToTypes<std::tuple<Types...>> {
  using Setter = TypeSetter<Types...>;
  using Getter = TypeGetter<Types...>;
};

} // namespace detail

/**
 * @class BlackboardPyWrapper
 * @brief A wrapper around the C++ Blackboard that stores Python objects and
 * native types.
 *
 * This wrapper provides a Python-friendly interface to the C++ Blackboard,
 * using template metaprogramming for efficient type detection during SET
 * operations and maintaining a type registry for safe GET operations.
 *
 * Supported types (checked in order during SET):
 * - Python str -> C++ std::string
 * - Python bool -> C++ bool
 * - Python int -> C++ int64_t
 * - Python float -> C++ double
 * - Python list/tuple -> C++ std::vector<py::object>
 * - Python dict -> C++ std::map<std::string, py::object>
 * - Other Python objects -> Stored as PyObject*
 */
class BlackboardPyWrapper {
private:
  std::shared_ptr<Blackboard> blackboard;
  // Type registry to track the C++ type of each stored value
  std::map<std::string, std::string> type_registry;

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
   * This method uses template metaprogramming to automatically detect
   * the type of the Python object and store it in the most appropriate
   * C++ representation. Type information is recorded for safe retrieval.
   *
   * @param key The key to associate with the value.
   * @param value The Python object to store.
   */
  void set(const std::string &key, py::object value) {
    // Determine the new type
    std::string new_type;

    // Handle None separately
    if (value.is_none()) {
      new_type = "PyObject";
    } else if (detail::PythonTypeChecker<std::string>::check(value)) {
      new_type = "string";
    } else if (detail::PythonTypeChecker<bool>::check(value)) {
      new_type = "bool";
    } else if (detail::PythonTypeChecker<int64_t>::check(value)) {
      new_type = "int64";
    } else if (detail::PythonTypeChecker<double>::check(value)) {
      new_type = "double";
    } else if (detail::PythonTypeChecker<std::vector<py::object>>::check(
                   value)) {
      new_type = "vector";
    } else if (detail::PythonTypeChecker<
                   std::map<std::string, py::object>>::check(value)) {
      new_type = "map";
    } else {
      new_type = "PyObject";
    }

    // If key exists and type is different, remove it first
    if (this->blackboard->contains(key)) {
      auto type_it = this->type_registry.find(key);
      if (type_it != this->type_registry.end() && type_it->second != new_type) {
        this->blackboard->remove(key);
        this->type_registry.erase(key);
      }
    }

    // Now set the new value
    if (new_type == "string") {
      this->blackboard->set<std::string>(
          key, detail::PythonTypeChecker<std::string>::extract(value));
    } else if (new_type == "bool") {
      this->blackboard->set<bool>(
          key, detail::PythonTypeChecker<bool>::extract(value));
    } else if (new_type == "int64") {
      this->blackboard->set<int64_t>(
          key, detail::PythonTypeChecker<int64_t>::extract(value));
    } else if (new_type == "double") {
      this->blackboard->set<double>(
          key, detail::PythonTypeChecker<double>::extract(value));
    } else if (new_type == "vector") {
      this->blackboard->set<std::vector<py::object>>(
          key,
          detail::PythonTypeChecker<std::vector<py::object>>::extract(value));
    } else if (new_type == "map") {
      this->blackboard->set<std::map<std::string, py::object>>(
          key,
          detail::PythonTypeChecker<std::map<std::string, py::object>>::extract(
              value));
    } else { // PyObject
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

    this->type_registry[key] = new_type;
  }

  /**
   * @brief Get a Python object from the blackboard.
   *
   * This method uses the type registry to retrieve values safely,
   * avoiding undefined behavior from incorrect type casts.
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

    // Use type registry to determine the correct type
    auto type_it = this->type_registry.find(key);
    if (type_it != this->type_registry.end()) {
      const std::string &type_name = type_it->second;

      if (type_name == "string") {
        return py::cast(this->blackboard->get<std::string>(key));
      } else if (type_name == "bool") {
        return py::cast(this->blackboard->get<bool>(key));
      } else if (type_name == "int64") {
        return py::cast(this->blackboard->get<int64_t>(key));
      } else if (type_name == "double") {
        return py::cast(this->blackboard->get<double>(key));
      } else if (type_name == "vector") {
        std::vector<py::object> value =
            this->blackboard->get<std::vector<py::object>>(key);
        py::list py_list;
        for (const auto &item : value) {
          py_list.append(item);
        }
        return py_list;
      } else if (type_name == "map") {
        std::map<std::string, py::object> value =
            this->blackboard->get<std::map<std::string, py::object>>(key);
        py::dict py_dict;
        for (const auto &item : value) {
          py_dict[py::str(item.first)] = item.second;
        }
        return py_dict;
      } else if (type_name == "PyObject") {
        BlackboardValuePyObject *wrapper =
            this->blackboard->get<BlackboardValuePyObject *>(key);
        PyObject *py_obj = wrapper->get();
        return py::reinterpret_borrow<py::object>(py_obj);
      }
    }

    // Fallback to original behavior for compatibility
    // (in case the blackboard was populated by C++ code without using this
    // wrapper)
    try {
      std::string value = this->blackboard->get<std::string>(key);
      return py::cast(value);
    } catch (const std::exception &) {
    }

    try {
      std::vector<py::object> value =
          this->blackboard->get<std::vector<py::object>>(key);
      py::list py_list;
      for (const auto &item : value) {
        py_list.append(item);
      }
      return py_list;
    } catch (const std::exception &) {
    }

    try {
      std::map<std::string, py::object> value =
          this->blackboard->get<std::map<std::string, py::object>>(key);
      py::dict py_dict;
      for (const auto &item : value) {
        py_dict[py::str(item.first)] = item.second;
      }
      return py_dict;
    } catch (const std::exception &) {
    }

    try {
      double value = this->blackboard->get<double>(key);
      return py::cast(value);
    } catch (const std::exception &) {
    }

    try {
      int64_t value = this->blackboard->get<int64_t>(key);
      return py::cast(value);
    } catch (const std::exception &) {
    }

    try {
      bool value = this->blackboard->get<bool>(key);
      return py::cast(value);
    } catch (const std::exception &) {
    }

    try {
      BlackboardValuePyObject *wrapper =
          this->blackboard->get<BlackboardValuePyObject *>(key);
      PyObject *py_obj = wrapper->get();
      return py::reinterpret_borrow<py::object>(py_obj);
    } catch (const std::exception &) {
    }

    throw std::runtime_error("Unable to retrieve value for key '" + key +
                             "' from blackboard");
  }

  /**
   * @brief Remove a value from the blackboard.
   * @param key The key associated with the value to remove.
   */
  void remove(const std::string &key) {
    this->blackboard->remove(key);
    this->type_registry.erase(key);
  }

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
