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
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_value.hpp"

namespace py = pybind11;

namespace yasmin {
namespace blackboard {

// Forward declaration
class BlackboardPyWrapper;

/**
 * @class BlackboardPyWrapper
 * @brief A wrapper around the C++ Blackboard that stores Python objects and
 * native types.
 *
 * This wrapper provides a Python-friendly interface to the C++ Blackboard,
 * automatically converting between C++ types (like std::string) and Python
 * objects when accessing blackboard values.
 */
class BlackboardPyWrapper {
private:
  /// @brief Underlying C++ Blackboard instance
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
   * @param key The key to associate with the value.
   * @param value The Python object to store.
   */
  void set(const std::string &key, py::object value) {
    if (py::isinstance<py::bool_>(value)) {
      this->blackboard->set<bool>(key, value.cast<bool>());
    } else if (py::isinstance<py::int_>(value)) {
      try {
        this->blackboard->set<int>(key, value.cast<int>());
      } catch (...) {
        this->blackboard->set<long>(key, value.cast<long>());
      }
    } else if (py::isinstance<py::float_>(value)) {
      this->blackboard->set<double>(key, value.cast<double>());
    } else if (py::isinstance<py::str>(value)) {
      this->blackboard->set<std::string>(key, value.cast<std::string>());
    } else if (py::isinstance<py::list>(value)) {
      this->blackboard->set<py::object>(key, value);
    } else if (py::isinstance<py::dict>(value)) {
      this->blackboard->set<py::object>(key, value);
    } else if (py::isinstance<py::tuple>(value)) {
      this->blackboard->set<py::object>(key, value);
    } else if (py::isinstance<py::set>(value)) {
      this->blackboard->set<py::object>(key, value);
    } else {
      this->blackboard->set<py::object>(key, value);
    }
  }

  /**
   * @brief Get a Python object from the blackboard.
   * @param key The key associated with the value.
   * @return The Python object.
   * @throws std::runtime_error if the key does not exist.
   */
  py::object get(const std::string &key) {
    // Get the type of the stored value
    std::string type = this->blackboard->get_type(key);

    // Check if it's a pybind11::object (Python object - includes all Python
    // types)
    if (type.find("pybind11::object") != std::string::npos ||
        type.find("pybind11::int_") != std::string::npos ||
        type.find("pybind11::float_") != std::string::npos ||
        type.find("pybind11::str") != std::string::npos ||
        type.find("pybind11::bool_") != std::string::npos ||
        type.find("pybind11::list") != std::string::npos ||
        type.find("pybind11::dict") != std::string::npos ||
        type.find("pybind11::set") != std::string::npos ||
        type.find("pybind11::tuple") != std::string::npos ||
        type.find("pybind11::bytes") != std::string::npos ||
        type.find("pybind11::none") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's a std::string (C++ string) - convert to Python str
    else if (type.find("std::string") != std::string::npos ||
             type.find("std::__cxx11::basic_string") != std::string::npos) {
      std::string cpp_value = this->blackboard->get<std::string>(key);
      return py::cast(cpp_value);
    }
    // Check if it's a std::vector (C++ vector) - convert to Python list
    else if (type.find("std::vector") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's a std::map (C++ map) - convert to Python dict
    else if (type.find("std::map") != std::string::npos ||
             type.find("std::unordered_map") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's a std::set (C++ set) - convert to Python set
    else if (type.find("std::set") != std::string::npos ||
             type.find("std::unordered_set") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's a std::list (C++ list) - convert to Python list
    else if (type.find("std::list") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's a std::tuple (C++ tuple) - convert to Python tuple
    else if (type.find("std::tuple") != std::string::npos) {
      return this->blackboard->get<py::object>(key);
    }
    // Check if it's an int (C++ int) - convert to Python int
    else if (type.find("int") != std::string::npos) {
      int cpp_value = this->blackboard->get<int>(key);
      return py::cast(cpp_value);
    }
    // Check if it's a long (C++ long) - convert to Python int
    else if (type.find("long") != std::string::npos) {
      long cpp_value = this->blackboard->get<long>(key);
      return py::cast(cpp_value);
    }
    // Check if it's a float or double (C++ float/double) - convert to Python
    // float
    else if (type.find("float") != std::string::npos ||
             type.find("double") != std::string::npos) {
      double cpp_value = this->blackboard->get<double>(key);
      return py::cast(cpp_value);
    }
    // Check if it's a bool (C++ bool) - convert to Python bool
    else if (type.find("bool") != std::string::npos) {
      bool cpp_value = this->blackboard->get<bool>(key);
      return py::cast(cpp_value);
    }
    // Default: try to get as py::object
    else {
      return this->blackboard->get<py::object>(key);
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
   * @brief Set the remappings of the blackboard.
   * @param remappings The remappings to set.
   */
  void set_remappings(const std::map<std::string, std::string> &remappings) {
    this->blackboard->set_remappings(remappings);
  }

  /**
   * @brief Get the remappings of the blackboard.
   * @return The remappings of the blackboard.
   */
  std::map<std::string, std::string> get_remappings() {
    return this->blackboard->get_remappings();
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
