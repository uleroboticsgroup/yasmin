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

#ifndef YASMIN__BLACKBOARD_PYWRAPPER_HPP_
#define YASMIN__BLACKBOARD_PYWRAPPER_HPP_

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "yasmin/blackboard.hpp"
#include "yasmin/callback_signal.hpp"
#include "yasmin/callback_signal_pyutils.hpp"
#include "yasmin/types.hpp"

namespace py = pybind11;

namespace yasmin {

/**
 * @class BlackboardPyWrapper
 * @brief Python-facing wrapper around the native Blackboard.
 *
 * The underlying Blackboard stores strongly typed C++ values. This wrapper
 * translates common Python types into matching C++ types on write and converts
 * them back into Python objects on read.
 *
 * Supported scalar mappings:
 * - bool                 <-> bool
 * - int                  <-> std::int64_t
 * - float                <-> double
 * - str                  <-> std::string
 * - bytes / bytearray    <-> std::vector<uint8_t>
 * - CallbackSignal       <-> std::shared_ptr<yasmin::CallbackSignal>
 *
 * Supported homogeneous container mappings:
 * - list[str] / tuple[str, ...]     <-> std::vector<std::string>
 * - list[int] / tuple[int, ...]     <-> std::vector<std::int64_t>
 * - list[float] / tuple[float, ...] <-> std::vector<double>
 * - list[bool] / tuple[bool, ...]   <-> std::vector<bool>
 * - dict[str, str]                  <-> std::unordered_map<std::string,
 * std::string>
 * - dict[str, int]                  <-> std::unordered_map<std::string,
 * std::int64_t>
 * - dict[str, float]                <-> std::unordered_map<std::string, double>
 * - dict[str, bool]                 <-> std::unordered_map<std::string, bool>
 *
 * Mixed containers, nested containers, and empty containers are stored as
 * py::object because no unique native C++ element type can be inferred safely.
 */
class BlackboardPyWrapper {
private:
  /// @brief Shared pointer to the underlying native blackboard.
  Blackboard::SharedPtr blackboard;

  /// @brief Alias for a typed vector of strings.
  using StringVector = std::vector<std::string>;
  /// @brief Alias for a typed vector of integers.
  using IntVector = std::vector<std::int64_t>;
  /// @brief Alias for a typed vector of floating point values.
  using FloatVector = std::vector<double>;
  /// @brief Alias for a typed vector of booleans.
  using BoolVector = std::vector<bool>;

  /// @brief Alias for a typed dictionary with string values.
  using StringDict = std::unordered_map<std::string, std::string>;
  /// @brief Alias for a typed dictionary with integer values.
  using IntDict = std::unordered_map<std::string, std::int64_t>;
  /// @brief Alias for a typed dictionary with floating point values.
  using FloatDict = std::unordered_map<std::string, double>;
  /// @brief Alias for a typed dictionary with boolean values.
  using BoolDict = std::unordered_map<std::string, bool>;

  /**
   * @brief Convert a Python bytes-like object into a byte vector.
   * @param value Python object expected to be bytes or bytearray.
   * @return Byte vector with the same raw contents.
   * @throws std::runtime_error if the object is not bytes-like.
   */
  static std::vector<uint8_t> py_buffer_to_vector(const py::handle &value) {
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
   * @brief Convert a C++ byte-like vector into Python bytes.
   * @tparam ByteT Element type of the input vector.
   * @param value Native byte buffer.
   * @return Python bytes object with identical raw contents.
   */
  template <typename ByteT>
  static py::bytes byte_vector_to_py_bytes(const std::vector<ByteT> &value) {
    return py::bytes(reinterpret_cast<const char *>(value.data()),
                     static_cast<py::ssize_t>(value.size()));
  }

  /**
   * @brief Check whether a Python object is bytes-like.
   * @param value Python object to test.
   * @return True if the object is bytes or bytearray.
   */
  static bool is_python_bytes_like(const py::handle &value) {
    return PyBytes_Check(value.ptr()) != 0 ||
           PyByteArray_Check(value.ptr()) != 0;
  }

  /**
   * @brief Check whether a Python object is an integer but not a boolean.
   * @param value Python object to test.
   * @return True if the object is an int and not a bool.
   *
   * Python bool is a subclass of int, therefore it must be filtered out
   * explicitly to avoid storing True and False as integers.
   */
  static bool is_python_int_like(const py::handle &value) {
    return py::isinstance<py::int_>(value) && !py::isinstance<py::bool_>(value);
  }

  /**
   * @brief Check whether a Python object is a floating point value.
   * @param value Python object to test.
   * @return True if the object is a Python float.
   */
  static bool is_python_float_like(const py::handle &value) {
    return py::isinstance<py::float_>(value);
  }

  /**
   * @brief Check whether a Python object is a numeric value.
   * @param value Python object to test.
   * @return True if the object is an int-like or float-like value.
   */
  static bool is_python_number_like(const py::handle &value) {
    return is_python_int_like(value) || is_python_float_like(value);
  }

  /**
   * @brief Check whether a Python object should be treated as a sequence.
   * @param value Python object to test.
   * @return True if the object is a list or tuple.
   */
  static bool is_python_sequence_like(const py::handle &value) {
    return py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value);
  }

  /**
   * @brief Check whether all elements of a Python sequence satisfy a predicate.
   * @tparam Predicate Callable returning bool for each element.
   * @param seq Python sequence to inspect.
   * @param pred Predicate applied to every element.
   * @return True if all elements satisfy the predicate.
   */
  template <typename Predicate>
  static bool sequence_matches(const py::sequence &seq, Predicate pred) {
    for (auto item : seq) {
      if (!pred(item)) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Convert a Python sequence into a typed C++ vector.
   * @tparam T Target element type.
   * @param seq Python sequence to convert.
   * @return Vector containing all converted elements.
   */
  template <typename T>
  static std::vector<T> sequence_to_vector(const py::sequence &seq) {
    std::vector<T> result;
    result.reserve(static_cast<std::size_t>(py::len(seq)));

    for (auto item : seq) {
      result.push_back(py::cast<T>(item));
    }

    return result;
  }

  /**
   * @brief Check whether all keys in a Python dict are strings.
   * @param dict Python dictionary to inspect.
   * @return True if all keys are Python strings.
   */
  static bool dict_has_only_string_keys(const py::dict &dict) {
    for (auto item : dict) {
      if (!py::isinstance<py::str>(item.first)) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Check whether all values of a Python dict satisfy a predicate.
   * @tparam Predicate Callable returning bool for each value.
   * @param dict Python dictionary to inspect.
   * @param pred Predicate applied to every value.
   * @return True if all values satisfy the predicate.
   */
  template <typename Predicate>
  static bool dict_values_match(const py::dict &dict, Predicate pred) {
    for (auto item : dict) {
      if (!pred(item.second)) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Convert a Python dict with string keys into a typed unordered_map.
   * @tparam T Target mapped type.
   * @param dict Python dictionary to convert.
   * @return Native unordered_map with converted keys and values.
   */
  template <typename T>
  static std::unordered_map<std::string, T>
  dict_to_unordered_map(const py::dict &dict) {
    std::unordered_map<std::string, T> result;
    result.reserve(static_cast<std::size_t>(py::len(dict)));

    for (auto item : dict) {
      result.emplace(py::cast<std::string>(item.first),
                     py::cast<T>(item.second));
    }

    return result;
  }

  /**
   * @brief Compare a stored type string against an exact C++ type.
   * @tparam T Type to compare against.
   * @param type Demangled type name stored in the blackboard.
   * @return True if the stored type exactly matches T.
   *
   * Exact matching avoids ambiguous substring checks such as "int" matching
   * both "int" and "std::vector<int>".
   */
  template <typename T> static bool is_exact_cpp_type(const std::string &type) {
    return type == demangle_type(typeid(T).name());
  }

public:
  /**
   * @brief Construct a wrapper with a newly created native Blackboard.
   */
  BlackboardPyWrapper() : blackboard(Blackboard::make_shared()) {}

  /**
   * @brief Construct a wrapper by moving an existing native Blackboard.
   * @param other Native blackboard instance to move into shared ownership.
   */
  BlackboardPyWrapper(Blackboard &&other)
      : blackboard(Blackboard::make_shared(std::move(other))) {}

  /**
   * @brief Construct a wrapper from an existing shared native Blackboard.
   * @param bb_ptr Shared pointer to a native Blackboard.
   */
  explicit BlackboardPyWrapper(Blackboard::SharedPtr bb_ptr)
      : blackboard(std::move(bb_ptr)) {}

  /**
   * @brief Copy constructor following native Blackboard copy semantics.
   * @param other Wrapper to copy from.
   *
   * The copied wrapper receives its own Blackboard handle created through the
   * native Blackboard copy constructor. This preserves shared value storage
   * while isolating remappings between the two Python objects.
   */
  BlackboardPyWrapper(const BlackboardPyWrapper &other)
      : blackboard(Blackboard::make_shared(*other.blackboard)) {}

  /**
   * @brief Copy assignment following native Blackboard copy semantics.
   * @param other Wrapper to copy from.
   * @return Reference to this wrapper.
   */
  BlackboardPyWrapper &operator=(const BlackboardPyWrapper &other) {
    if (this != &other) {
      this->blackboard = Blackboard::make_shared(*other.blackboard);
    }
    return *this;
  }

  /**
   * @brief Store a Python object in the blackboard.
   * @param key Blackboard key.
   * @param value Python object to store.
   *
   * The wrapper first tries to map the value to a native C++ type. If no safe
   * native mapping exists, the value is stored as py::object.
   */
  void set(const std::string &key, py::object value) {
    if (py::isinstance<py::bool_>(value)) {
      this->blackboard->set<bool>(key, value.cast<bool>());
      return;
    }

    if (is_python_int_like(value)) {
      this->blackboard->set<std::int64_t>(key, value.cast<std::int64_t>());
      return;
    }

    if (py::isinstance<py::float_>(value)) {
      this->blackboard->set<double>(key, value.cast<double>());
      return;
    }

    if (is_python_bytes_like(value)) {
      this->blackboard->set<std::vector<uint8_t>>(key,
                                                  py_buffer_to_vector(value));
      return;
    }

    if (py::isinstance<py::str>(value)) {
      this->blackboard->set<std::string>(key, value.cast<std::string>());
      return;
    }

    if (yasmin::callback_signal_pyutils::is_python_callback_signal_like(
            value)) {
      this->blackboard->set<yasmin::CallbackSignal::SharedPtr>(
          key,
          yasmin::callback_signal_pyutils::cast_python_callback_signal(value));
      return;
    }

    if (is_python_sequence_like(value)) {
      py::sequence seq = value.cast<py::sequence>();

      // Empty sequences do not carry enough type information to infer a native
      // element type safely.
      if (py::len(seq) == 0) {
        this->blackboard->set<py::object>(key, value);
        return;
      }

      // Bool must be checked before int because Python bool is also an int.
      if (sequence_matches(seq, [](const py::handle &item) {
            return py::isinstance<py::bool_>(item);
          })) {
        this->blackboard->set<BoolVector>(key, sequence_to_vector<bool>(seq));
        return;
      }

      if (sequence_matches(seq, [](const py::handle &item) {
            return is_python_int_like(item);
          })) {
        this->blackboard->set<IntVector>(key,
                                         sequence_to_vector<std::int64_t>(seq));
        return;
      }

      // Mixed int/float sequences are normalized to double.
      if (sequence_matches(seq, [](const py::handle &item) {
            return is_python_number_like(item);
          })) {
        this->blackboard->set<FloatVector>(key,
                                           sequence_to_vector<double>(seq));
        return;
      }

      if (sequence_matches(seq, [](const py::handle &item) {
            return py::isinstance<py::str>(item);
          })) {
        this->blackboard->set<StringVector>(
            key, sequence_to_vector<std::string>(seq));
        return;
      }

      this->blackboard->set<py::object>(key, value);
      return;
    }

    if (py::isinstance<py::dict>(value)) {
      py::dict dict = value.cast<py::dict>();

      // Only dict[str, T] with a homogeneous value type is converted into a
      // native unordered_map.
      if (py::len(dict) == 0 || !dict_has_only_string_keys(dict)) {
        this->blackboard->set<py::object>(key, value);
        return;
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return py::isinstance<py::bool_>(item);
          })) {
        this->blackboard->set<BoolDict>(key, dict_to_unordered_map<bool>(dict));
        return;
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return is_python_int_like(item);
          })) {
        this->blackboard->set<IntDict>(
            key, dict_to_unordered_map<std::int64_t>(dict));
        return;
      }

      // Mixed int/float dictionaries are normalized to double.
      if (dict_values_match(dict, [](const py::handle &item) {
            return is_python_number_like(item);
          })) {
        this->blackboard->set<FloatDict>(key,
                                         dict_to_unordered_map<double>(dict));
        return;
      }

      if (dict_values_match(dict, [](const py::handle &item) {
            return py::isinstance<py::str>(item);
          })) {
        this->blackboard->set<StringDict>(
            key, dict_to_unordered_map<std::string>(dict));
        return;
      }

      this->blackboard->set<py::object>(key, value);
      return;
    }

    this->blackboard->set<py::object>(key, value);
  }

  /**
   * @brief Retrieve a Python object from the blackboard.
   * @param key Blackboard key.
   * @return Python object reconstructed from the stored native value.
   *
   * The native blackboard stores the type name of each entry. This method uses
   * that exact stored type information to select the matching conversion path.
   */
  py::object get(const std::string &key) const {
    const std::string type = this->blackboard->get_type(key);

    if (is_exact_cpp_type<std::vector<uint8_t>>(type)) {
      return byte_vector_to_py_bytes(
          this->blackboard->get<std::vector<uint8_t>>(key));
    }

    if (is_exact_cpp_type<std::vector<unsigned char>>(type)) {
      return byte_vector_to_py_bytes(
          this->blackboard->get<std::vector<unsigned char>>(key));
    }

    if (is_exact_cpp_type<std::vector<char>>(type)) {
      return byte_vector_to_py_bytes(
          this->blackboard->get<std::vector<char>>(key));
    }

    if (is_exact_cpp_type<StringVector>(type)) {
      return py::cast(this->blackboard->get<StringVector>(key));
    }

    if (is_exact_cpp_type<IntVector>(type)) {
      return py::cast(this->blackboard->get<IntVector>(key));
    }

    if (is_exact_cpp_type<std::vector<int>>(type)) {
      return py::cast(this->blackboard->get<std::vector<int>>(key));
    }

    if (is_exact_cpp_type<std::vector<long>>(type)) {
      return py::cast(this->blackboard->get<std::vector<long>>(key));
    }

    if (is_exact_cpp_type<std::vector<long long>>(type)) {
      return py::cast(this->blackboard->get<std::vector<long long>>(key));
    }

    if (is_exact_cpp_type<FloatVector>(type)) {
      return py::cast(this->blackboard->get<FloatVector>(key));
    }

    if (is_exact_cpp_type<std::vector<float>>(type)) {
      return py::cast(this->blackboard->get<std::vector<float>>(key));
    }

    if (is_exact_cpp_type<BoolVector>(type)) {
      return py::cast(this->blackboard->get<BoolVector>(key));
    }

    if (is_exact_cpp_type<StringDict>(type)) {
      return py::cast(this->blackboard->get<StringDict>(key));
    }

    if (is_exact_cpp_type<IntDict>(type)) {
      return py::cast(this->blackboard->get<IntDict>(key));
    }

    if (is_exact_cpp_type<std::unordered_map<std::string, int>>(type)) {
      return py::cast(
          this->blackboard->get<std::unordered_map<std::string, int>>(key));
    }

    if (is_exact_cpp_type<std::unordered_map<std::string, long>>(type)) {
      return py::cast(
          this->blackboard->get<std::unordered_map<std::string, long>>(key));
    }

    if (is_exact_cpp_type<std::unordered_map<std::string, long long>>(type)) {
      return py::cast(
          this->blackboard->get<std::unordered_map<std::string, long long>>(
              key));
    }

    if (is_exact_cpp_type<FloatDict>(type)) {
      return py::cast(this->blackboard->get<FloatDict>(key));
    }

    if (is_exact_cpp_type<std::unordered_map<std::string, float>>(type)) {
      return py::cast(
          this->blackboard->get<std::unordered_map<std::string, float>>(key));
    }

    if (is_exact_cpp_type<BoolDict>(type)) {
      return py::cast(this->blackboard->get<BoolDict>(key));
    }

    if (is_exact_cpp_type<yasmin::CallbackSignal::SharedPtr>(type)) {
      return py::cast(
          this->blackboard->get<yasmin::CallbackSignal::SharedPtr>(key));
    }

    if (is_exact_cpp_type<std::string>(type)) {
      return py::cast(this->blackboard->get<std::string>(key));
    }

    if (is_exact_cpp_type<std::int64_t>(type)) {
      return py::cast(this->blackboard->get<std::int64_t>(key));
    }

    if (is_exact_cpp_type<int>(type)) {
      return py::cast(this->blackboard->get<int>(key));
    }

    if (is_exact_cpp_type<long>(type)) {
      return py::cast(this->blackboard->get<long>(key));
    }

    if (is_exact_cpp_type<float>(type)) {
      return py::cast(this->blackboard->get<float>(key));
    }

    if (is_exact_cpp_type<double>(type)) {
      return py::cast(this->blackboard->get<double>(key));
    }

    if (is_exact_cpp_type<bool>(type)) {
      return py::cast(this->blackboard->get<bool>(key));
    }

    if (is_exact_cpp_type<py::object>(type)) {
      return this->blackboard->get<py::object>(key);
    }

    return py::none();
  }

  /**
   * @brief Remove a value from the blackboard.
   * @param key Blackboard key to remove.
   */
  void remove(const std::string &key) { this->blackboard->remove(key); }

  /**
   * @brief Check whether a key exists in the blackboard.
   * @param key Blackboard key to test.
   * @return True if the key exists.
   */
  bool contains(const std::string &key) const {
    return this->blackboard->contains(key);
  }

  /**
   * @brief Get the number of entries in the blackboard.
   * @return Number of stored key-value pairs.
   */
  int size() const { return this->blackboard->size(); }

  /**
   * @brief Get the keys visible in the current remapping scope.
   * @return Sorted list of visible key names.
   */
  std::vector<std::string> keys() const { return this->blackboard->keys(); }

  /**
   * @brief Get the values visible in the current remapping scope.
   * @return Python list with the values in key order.
   */
  py::list values() const {
    py::list result;

    for (const auto &key : this->blackboard->keys()) {
      result.append(this->get(key));
    }

    return result;
  }

  /**
   * @brief Get the key-value pairs visible in the current remapping scope.
   * @return Python list of ``(key, value)`` tuples.
   */
  py::list items() const {
    py::list result;

    for (const auto &key : this->blackboard->keys()) {
      result.append(py::make_tuple(key, this->get(key)));
    }

    return result;
  }

  /**
   * @brief Get a string representation of the blackboard contents.
   * @return Human-readable description of the stored keys and types.
   */
  std::string to_string() const { return this->blackboard->to_string(); }

  /**
   * @brief Set key remappings on the underlying blackboard.
   * @param remappings Remapping table to apply.
   */
  void set_remappings(const Remappings &remappings) {
    this->blackboard->set_remappings(remappings);
  }

  /**
   * @brief Get the current key remappings.
   * @return Reference to the underlying remapping table.
   */
  const Remappings &get_remappings() const {
    return this->blackboard->get_remappings();
  }

  /**
   * @brief Access the underlying native blackboard.
   * @return Shared pointer to the native blackboard instance.
   */
  Blackboard::SharedPtr get_cpp_blackboard() const { return this->blackboard; }
};

} // namespace yasmin

#endif // YASMIN__BLACKBOARD_PYWRAPPER_HPP_
