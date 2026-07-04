// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_
#define YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "yasmin/blackboard_key_info.hpp"
#include "yasmin/callback_signal.hpp"
#include "yasmin/callback_signal_pyutils.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

namespace yasmin {

namespace detail {

/// @brief Alias for a vector of strings.
using StringVector = std::vector<std::string>;
/// @brief Alias for a vector of 64-bit integers.
using IntVector = std::vector<std::int64_t>;
/// @brief Alias for a vector of doubles.
using FloatVector = std::vector<double>;
/// @brief Alias for a vector of booleans.
using BoolVector = std::vector<bool>;

/// @brief Alias for a dictionary mapping strings to strings.
using StringDict = std::unordered_map<std::string, std::string>;
/// @brief Alias for a dictionary mapping strings to 64-bit integers.
using IntDict = std::unordered_map<std::string, std::int64_t>;
/// @brief Alias for a dictionary mapping strings to doubles.
using FloatDict = std::unordered_map<std::string, double>;
/// @brief Alias for a dictionary mapping strings to booleans.
using BoolDict = std::unordered_map<std::string, bool>;

/**
 * @brief Convert a Python bytes or bytearray object to a vector of uint8_t.
 * @param value The Python handle to convert.
 * @param size Internal usage for PyBytes_AsStringAndSize.
 * @return A vector of uint8_t containing the buffer data.
 */
inline std::vector<uint8_t> py_buffer_to_vector(const py::handle &value) {
  char *data = nullptr;
  py::ssize_t size = 0;

  if (PyBytes_Check(value.ptr()) != 0) {
    if (PyBytes_AsStringAndSize(value.ptr(), &data, &size) != 0) {
      throw py::error_already_set();
    }
  } else if (PyByteArray_Check(value.ptr()) != 0) {
    data = PyByteArray_AsString(value.ptr());
    size = PyByteArray_Size(value.ptr());
  } else {
    throw std::runtime_error("Expected bytes or bytearray object");
  }

  return std::vector<uint8_t>(reinterpret_cast<uint8_t *>(data),
                              reinterpret_cast<uint8_t *>(data) + size);
}

/**
 * @brief Convert a vector of byte-like data to a Python bytes object.
 * @tparam T The element type of the vector.
 * @param vec The vector to convert.
 * @return A py::bytes object containing the vector data.
 */
template <typename ByteT>
inline py::bytes byte_vector_to_py_bytes(const std::vector<ByteT> &vec) {
  return py::bytes(reinterpret_cast<const char *>(vec.data()),
                   static_cast<py::ssize_t>(vec.size()));
}

/**
 * @brief Check if a Python object is bytes-like (bytes or bytearray).
 * @param value The Python handle to check.
 * @return True if the object is a bytes or bytearray instance.
 */
inline bool is_python_bytes_like(const py::handle &value) {
  return PyBytes_Check(value.ptr()) != 0 || PyByteArray_Check(value.ptr()) != 0;
}

/**
 * @brief Check if a Python object is an integer (but not a bool).
 * @param value The Python handle to check.
 * @return True if the object is an int instance excluding bool.
 */
inline bool is_python_int_like(const py::handle &value) {
  return py::isinstance<py::int_>(value) && !py::isinstance<py::bool_>(value);
}

/**
 * @brief Check if a Python object is a float.
 * @param value The Python handle to check.
 * @return True if the object is a float instance.
 */
inline bool is_python_float_like(const py::handle &value) {
  return py::isinstance<py::float_>(value);
}

/**
 * @brief Check if a Python object is a number (int or float).
 * @param value The Python handle to check.
 * @return True if the object is an int or float instance.
 */
inline bool is_python_number_like(const py::handle &value) {
  return is_python_int_like(value) || is_python_float_like(value);
}

/**
 * @brief Check if a Python object is a sequence (list or tuple).
 * @param value The Python handle to check.
 * @return True if the object is a list or tuple instance.
 */
inline bool is_python_sequence_like(const py::handle &value) {
  return py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value);
}

/**
 * @brief Check if every element of a sequence satisfies a predicate.
 * @tparam Predicate The callable type used to check elements.
 * @param seq The Python sequence to check.
 * @param pred The predicate applied to each element.
 * @return True if all elements pass the predicate.
 */
template <typename Predicate>
inline bool sequence_matches(const py::sequence &seq, Predicate pred) {
  for (auto item : seq) {
    if (!pred(item)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Convert a Python sequence to a vector of the specified type.
 * @tparam T The element type of the resulting vector.
 * @param seq The Python sequence to convert.
 * @return A vector containing the cast elements.
 */
template <typename T>
inline std::vector<T> sequence_to_vector(const py::sequence &seq) {
  std::vector<T> result;
  result.reserve(static_cast<std::size_t>(py::len(seq)));

  for (auto item : seq) {
    result.push_back(py::cast<T>(item));
  }

  return result;
}

/**
 * @brief Check if a Python dictionary has only string keys.
 * @param dict The Python dictionary to check.
 * @return True if all keys are strings.
 */
inline bool dict_has_only_string_keys(const py::dict &dict) {
  for (auto item : dict) {
    if (!py::isinstance<py::str>(item.first)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check if every value in a Python dictionary satisfies a predicate.
 * @tparam Predicate The callable type used to check values.
 * @param dict The Python dictionary to check.
 * @param pred The predicate applied to each value.
 * @return True if all values pass the predicate.
 */
template <typename Predicate>
inline bool dict_values_match(const py::dict &dict, Predicate pred) {
  for (auto item : dict) {
    if (!pred(item.second)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Convert a Python dictionary to an unordered_map with string keys.
 * @tparam T The value type of the resulting map.
 * @param dict The Python dictionary to convert.
 * @return An unordered_map containing the cast key-value pairs.
 */
template <typename T>
inline std::unordered_map<std::string, T>
dict_to_unordered_map(const py::dict &dict) {
  std::unordered_map<std::string, T> result;
  result.reserve(static_cast<std::size_t>(py::len(dict)));

  for (auto item : dict) {
    result.emplace(py::cast<std::string>(item.first), py::cast<T>(item.second));
  }

  return result;
}

/**
 * @brief Check if a demangled type name matches the exact C++ type T.
 * @tparam T The C++ type to compare against.
 * @param type The demangled type name string.
 * @return True if the type matches exactly.
 */
template <typename T> inline bool is_exact_cpp_type(const std::string &type) {
  return type == demangle_type(typeid(T).name());
}

} // namespace detail

/**
 * @brief Create a BlackboardKeyInfo from a Python object default value.
 *
 * The conversion rules mirror the BlackboardPyWrapper so metadata created from
 * Python states uses the same native storage types as values written to the
 * runtime blackboard.
 * @param key_name The name of the blackboard key.
 * @param value The Python object representing the default value.
 * @return A BlackboardKeyInfo with the detected type and default value.
 */
inline BlackboardKeyInfo
blackboard_key_info_from_pyobject(const std::string &key_name,
                                  py::object value) {
  using namespace detail;

  if (py::isinstance<py::bool_>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<bool>());
  }

  if (is_python_int_like(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<std::int64_t>());
  }

  if (py::isinstance<py::float_>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<double>());
  }

  if (is_python_bytes_like(value)) {
    return BlackboardKeyInfo(key_name, "", py_buffer_to_vector(value));
  }

  if (py::isinstance<py::str>(value)) {
    return BlackboardKeyInfo(key_name, "", value.cast<std::string>());
  }

  if (yasmin::callback_signal_pyutils::is_python_callback_signal_like(value)) {
    return BlackboardKeyInfo(
        key_name, "",
        yasmin::callback_signal_pyutils::cast_python_callback_signal(value));
  }

  if (is_python_sequence_like(value)) {
    py::sequence seq = value.cast<py::sequence>();

    if (py::len(seq) == 0) {
      BlackboardKeyInfo info(key_name);
      info.has_default = true;
      info.default_value_type = "py::object";
      auto stored = std::make_shared<py::object>(value);
      info.default_value = stored;
      info.inject_default = [stored](Blackboard &bb, const std::string &key) {
        py::gil_scoped_acquire gil;
        bb.set<py::object>(key, *stored);
      };
      return info;
    }

    if (sequence_matches(seq, [](const py::handle &item) {
          return py::isinstance<py::bool_>(item);
        })) {
      return BlackboardKeyInfo(key_name, "", sequence_to_vector<bool>(seq));
    }

    if (sequence_matches(seq, [](const py::handle &item) {
          return is_python_int_like(item);
        })) {
      return BlackboardKeyInfo(key_name, "",
                               sequence_to_vector<std::int64_t>(seq));
    }

    if (sequence_matches(seq, [](const py::handle &item) {
          return is_python_number_like(item);
        })) {
      return BlackboardKeyInfo(key_name, "", sequence_to_vector<double>(seq));
    }

    if (sequence_matches(seq, [](const py::handle &item) {
          return py::isinstance<py::str>(item);
        })) {
      return BlackboardKeyInfo(key_name, "",
                               sequence_to_vector<std::string>(seq));
    }
  }

  if (py::isinstance<py::dict>(value)) {
    py::dict dict = value.cast<py::dict>();

    if (py::len(dict) != 0 && dict_has_only_string_keys(dict)) {
      if (dict_values_match(dict, [](const py::handle &item) {
            return py::isinstance<py::bool_>(item);
          })) {
        return BlackboardKeyInfo(key_name, "",
                                 dict_to_unordered_map<bool>(dict));
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return is_python_int_like(item);
          })) {
        return BlackboardKeyInfo(key_name, "",
                                 dict_to_unordered_map<std::int64_t>(dict));
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return is_python_number_like(item);
          })) {
        return BlackboardKeyInfo(key_name, "",
                                 dict_to_unordered_map<double>(dict));
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return py::isinstance<py::str>(item);
          })) {
        return BlackboardKeyInfo(key_name, "",
                                 dict_to_unordered_map<std::string>(dict));
      }
    }
  }

  BlackboardKeyInfo info(key_name);
  info.has_default = true;
  info.default_value_type = "py::object";

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
 * Exact type matching is used so container metadata remains unambiguous when
 * scalar and container types share substrings such as int and std::vector<int>.
 * @param info The BlackboardKeyInfo whose default value to convert.
 * @return The default value as a Python object.
 * @throws std::runtime_error If the stored type is not supported for Python
 * conversion.
 */
inline py::object
blackboard_key_info_get_py_default(const BlackboardKeyInfo &info) {
  using namespace detail;

  if (!info.has_default) {
    return py::none();
  }

  const std::string &type = info.default_value_type;

  if (is_exact_cpp_type<std::vector<uint8_t>>(type)) {
    return byte_vector_to_py_bytes(
        info.get_default_value<std::vector<uint8_t>>());
  }
  if (is_exact_cpp_type<std::vector<unsigned char>>(type)) {
    return byte_vector_to_py_bytes(
        info.get_default_value<std::vector<unsigned char>>());
  }
  if (is_exact_cpp_type<std::vector<char>>(type)) {
    return byte_vector_to_py_bytes(info.get_default_value<std::vector<char>>());
  }

  if (is_exact_cpp_type<StringVector>(type)) {
    return py::cast(info.get_default_value<StringVector>());
  }
  if (is_exact_cpp_type<IntVector>(type) ||
      is_exact_cpp_type<std::vector<int>>(type) ||
      is_exact_cpp_type<std::vector<long>>(type) ||
      is_exact_cpp_type<std::vector<long long>>(type)) {
    if (is_exact_cpp_type<IntVector>(type)) {
      return py::cast(info.get_default_value<IntVector>());
    }
    if (is_exact_cpp_type<std::vector<int>>(type)) {
      return py::cast(info.get_default_value<std::vector<int>>());
    }
    if (is_exact_cpp_type<std::vector<long>>(type)) {
      return py::cast(info.get_default_value<std::vector<long>>());
    }
    return py::cast(info.get_default_value<std::vector<long long>>());
  }
  if (is_exact_cpp_type<FloatVector>(type) ||
      is_exact_cpp_type<std::vector<float>>(type)) {
    if (is_exact_cpp_type<FloatVector>(type)) {
      return py::cast(info.get_default_value<FloatVector>());
    }
    return py::cast(info.get_default_value<std::vector<float>>());
  }
  if (is_exact_cpp_type<BoolVector>(type)) {
    return py::cast(info.get_default_value<BoolVector>());
  }

  if (is_exact_cpp_type<StringDict>(type)) {
    return py::cast(info.get_default_value<StringDict>());
  }
  if (is_exact_cpp_type<IntDict>(type) ||
      is_exact_cpp_type<std::unordered_map<std::string, int>>(type) ||
      is_exact_cpp_type<std::unordered_map<std::string, long>>(type) ||
      is_exact_cpp_type<std::unordered_map<std::string, long long>>(type)) {
    if (is_exact_cpp_type<IntDict>(type)) {
      return py::cast(info.get_default_value<IntDict>());
    }
    if (is_exact_cpp_type<std::unordered_map<std::string, int>>(type)) {
      return py::cast(
          info.get_default_value<std::unordered_map<std::string, int>>());
    }
    if (is_exact_cpp_type<std::unordered_map<std::string, long>>(type)) {
      return py::cast(
          info.get_default_value<std::unordered_map<std::string, long>>());
    }
    return py::cast(
        info.get_default_value<std::unordered_map<std::string, long long>>());
  }
  if (is_exact_cpp_type<FloatDict>(type) ||
      is_exact_cpp_type<std::unordered_map<std::string, float>>(type)) {
    if (is_exact_cpp_type<FloatDict>(type)) {
      return py::cast(info.get_default_value<FloatDict>());
    }
    return py::cast(
        info.get_default_value<std::unordered_map<std::string, float>>());
  }
  if (is_exact_cpp_type<BoolDict>(type)) {
    return py::cast(info.get_default_value<BoolDict>());
  }

  if (is_exact_cpp_type<yasmin::CallbackSignal::SharedPtr>(type)) {
    return py::cast(
        info.get_default_value<yasmin::CallbackSignal::SharedPtr>());
  }

  if (is_exact_cpp_type<std::string>(type)) {
    return py::cast(info.get_default_value<std::string>());
  }
  if (is_exact_cpp_type<std::int64_t>(type)) {
    return py::cast(info.get_default_value<std::int64_t>());
  }
  if (is_exact_cpp_type<int>(type)) {
    return py::cast(info.get_default_value<int>());
  }
  if (is_exact_cpp_type<long>(type)) {
    return py::cast(info.get_default_value<long>());
  }
  if (is_exact_cpp_type<long long>(type)) {
    return py::cast(info.get_default_value<long long>());
  }
  if (is_exact_cpp_type<float>(type)) {
    return py::cast(info.get_default_value<float>());
  }
  if (is_exact_cpp_type<double>(type)) {
    return py::cast(info.get_default_value<double>());
  }
  if (is_exact_cpp_type<bool>(type)) {
    return py::cast(info.get_default_value<bool>());
  }
  if (type == "py::object") {
    return info.get_default_value<py::object>();
  }

  throw std::runtime_error("Unsupported BlackboardKeyInfo default value type "
                           "for Python conversion: " +
                           type);
}

} // namespace yasmin

namespace yasmin {

/**
 * @brief Extension methods for BlackboardKeyInfo.
 * Added via ADL-friendly free functions used by the pybind code.
 */
struct BlackboardKeyInfoPy {
  /**
   * @brief Create a BlackboardKeyInfo from a key name and Python value.
   * @param key_name The name of the blackboard key.
   * @param value The Python object default value.
   * @return A BlackboardKeyInfo with the detected type and default.
   */
  static BlackboardKeyInfo from_pyobject(const std::string &key_name,
                                         py::object value) {
    return blackboard_key_info_from_pyobject(key_name, value);
  }

  /**
   * @brief Extract the Python default value from a BlackboardKeyInfo.
   * @param info The BlackboardKeyInfo to extract from.
   * @return The default value as a Python object.
   */
  static py::object get_py_default_value(const BlackboardKeyInfo &info) {
    return blackboard_key_info_get_py_default(info);
  }
};

} // namespace yasmin

#endif // YASMIN__BLACKBOARD_KEY_INFO_PY_HPP_
