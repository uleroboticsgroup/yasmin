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

#include <map>
#include <pybind11/pybind11.h>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_value_pyobject.hpp"

namespace py = pybind11;

namespace yasmin {
namespace blackboard {

/**
 * @class BlackboardPyWrapper
 * @brief A wrapper around the C++ Blackboard that stores Python objects.
 *
 * This wrapper provides a Python-friendly interface to the C++ Blackboard,
 * allowing storage and retrieval of arbitrary Python objects while
 * maintaining thread safety and proper memory management.
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
   * @param key The key to associate with the value.
   * @param value The Python object to store.
   */
  void set(const std::string &key, py::object value) {
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
   * @param key The key associated with the value.
   * @return The Python object.
   * @throws std::runtime_error if the key does not exist.
   */
  py::object get(const std::string &key) {
    BlackboardValuePyObject *wrapper =
        this->blackboard->get<BlackboardValuePyObject *>(key);
    PyObject *py_obj = wrapper->get();
    return py::reinterpret_borrow<py::object>(py_obj);
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
