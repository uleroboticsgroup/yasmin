// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_
#define YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>

#include "yasmin/blackboard_key_info.hpp"
#include "yasmin/state.hpp"

namespace py = pybind11;

namespace yasmin {

/**
 * @brief Create a BlackboardKeyInfo from a Python object default value.
 *
 * Detects the Python type and stores the value using the appropriate C++ type,
 * similar to BlackboardPyWrapper::set. Supports bool, int, long, double,
 * str, bytes, and arbitrary py::object.
 */
inline BlackboardKeyInfo
blackboard_key_info_from_pyobject(const std::string &key_name,
                                  py::object value) {
  if (py::isinstance<py::bool_>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<bool>());
  } else if (py::isinstance<py::int_>(value)) {
    try {
      return BlackboardKeyInfo(key_name, "", value.cast<int>());
    } catch (...) {
      return BlackboardKeyInfo(key_name, "", value.cast<long>());
    }
  } else if (py::isinstance<py::float_>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<double>());
  } else if (py::isinstance<py::str>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<std::string>());
  } else if (py::isinstance<py::bytes>(value)) {
    std::string data = value.cast<std::string>();
    return BlackboardKeyInfo(key_name, "",
                             std::vector<uint8_t>(data.begin(), data.end()));
  }
  BlackboardKeyInfo info(key_name);
  info.has_default = true;
  info.default_value_type = "py::object";

  // Keep the Python object alive via a shared_ptr to py::object.
  auto stored = std::make_shared<py::object>(value);
  info.default_value = stored;
  info.inject_default = [stored](Blackboard &bb, const std::string &key) {
    py::gil_scoped_acquire gil;
    bb.set<py::object>(key, *stored);
  };

  return info;
}

/**
 * @brief Convert a BlackboardKeyInfo default value back to a Python object.
 *
 * Inspects the stored type name and casts accordingly. For types stored as
 * py::object the value is returned directly.
 */
inline py::object
blackboard_key_info_get_py_default(const BlackboardKeyInfo &info) {
  if (!info.has_default) {
    return py::none();
  }

  const std::string &type = info.default_value_type;

  if (type.find("bool") != std::string::npos) {
    return py::cast(info.get_default_value<bool>());
  } else if (type.find("long") != std::string::npos) {
    return py::cast(info.get_default_value<long>());
  } else if (type.find("int") != std::string::npos) {
    return py::cast(info.get_default_value<int>());
  } else if (type.find("double") != std::string::npos ||
             type.find("float") != std::string::npos) {
    return py::cast(info.get_default_value<double>());
  } else if (type.find("std::string") != std::string::npos ||
             type.find("std::__cxx11::basic_string") != std::string::npos) {
    return py::cast(info.get_default_value<std::string>());
  } else if (type.find("std::vector<unsigned char") != std::string::npos ||
             type.find("std::vector<uint8_t") != std::string::npos) {
    auto vec = info.get_default_value<std::vector<uint8_t>>();
    return py::bytes(reinterpret_cast<const char *>(vec.data()),
                     static_cast<py::ssize_t>(vec.size()));
  } else if (type == "py::object") {
    return info.get_default_value<py::object>();
  } else {
    throw std::runtime_error("Unsupported BlackboardKeyInfo default value type "
                             "for Python conversion: " +
                             type);
  }
}

} // namespace yasmin

// Add convenience static/member access for BlackboardKeyInfo
namespace yasmin {

/**
 * @brief Extension methods for BlackboardKeyInfo.
 * Added via ADL-friendly free functions used by the pybind code.
 */
struct BlackboardKeyInfoPy {
  static BlackboardKeyInfo from_pyobject(const std::string &key_name,
                                         py::object value) {
    return blackboard_key_info_from_pyobject(key_name, value);
  }

  static py::object get_py_default_value(const BlackboardKeyInfo &info) {
    return blackboard_key_info_get_py_default(info);
  }
};

} // namespace yasmin

#endif // YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_
