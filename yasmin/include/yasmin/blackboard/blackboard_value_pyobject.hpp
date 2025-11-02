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

#ifndef YASMIN__BLACKBOARD__BLACKBOARD_VALUE_PYOBJECT_HPP
#define YASMIN__BLACKBOARD__BLACKBOARD_VALUE_PYOBJECT_HPP

#include <Python.h>
#include <string>

#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

/**
 * @class BlackboardValuePyObject
 * @brief A specialization of BlackboardValue for Python objects.
 *
 * This class wraps Python objects (PyObject*) and properly manages
 * their reference counting to prevent memory leaks and ensure
 * proper lifetime management when stored in the blackboard.
 */
class BlackboardValuePyObject : public BlackboardValueInterface {
private:
  /// The stored Python object.
  PyObject *value;

public:
  /**
   * @brief Constructs a BlackboardValuePyObject with the specified PyObject.
   * @param value The Python object to store. Its reference count will be
   * incremented.
   */
  BlackboardValuePyObject(PyObject *value) : value(value) {
    Py_XINCREF(this->value); // Increment reference count
  }

  /**
   * @brief Destructor that properly releases the Python object.
   */
  ~BlackboardValuePyObject() {
    Py_XDECREF(this->value); // Decrement reference count
  }

  /**
   * @brief Retrieve the stored Python object.
   * @return The stored PyObject*.
   */
  PyObject *get() { return this->value; }

  /**
   * @brief Set a new Python object.
   * @param value The new Python object to store.
   */
  void set(PyObject *value) {
    PyObject *old_value = this->value;
    this->value = value;
    Py_XINCREF(this->value); // Increment new value's ref count
    Py_XDECREF(old_value);   // Decrement old value's ref count
  }

  /**
   * @brief Get the type of the stored Python object as a string.
   * @return A string representation of the Python object's type.
   */
  std::string get_type() {
    if (this->value == nullptr) {
      return "None";
    }

    PyObject *type_obj = PyObject_Type(this->value);
    if (type_obj == nullptr) {
      return "Unknown";
    }

    PyObject *type_name = PyObject_GetAttrString(type_obj, "__name__");
    Py_DECREF(type_obj);

    if (type_name == nullptr) {
      return "Unknown";
    }

    const char *name_str = PyUnicode_AsUTF8(type_name);
    std::string result = (name_str != nullptr) ? name_str : "Unknown";
    Py_DECREF(type_name);

    return result;
  }

  /**
   * @brief Convert the stored value's type information to a string.
   * @return A string representation of the type of the stored value.
   */
  std::string to_string() override { return this->get_type(); }
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN__BLACKBOARD__BLACKBOARD_VALUE_PYOBJECT_HPP
