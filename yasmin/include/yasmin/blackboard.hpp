// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#ifndef YASMIN__BLACKBOARD_HPP
#define YASMIN__BLACKBOARD_HPP

#include <cxxabi.h>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "yasmin/logs.hpp"
#include "yasmin/types.hpp"

namespace yasmin {

/**
 * @brief Demangle a C++ type name to a human-readable format.
 * @param mangled_name The mangled type name.
 * @return The demangled type name.
 */
inline std::string demangle_type(const std::string &mangled_name) {

  std::string name = mangled_name;

#ifdef __GNUG__ // If using GCC/G++
  int status;
  // Demangle the name using GCC's demangling function
  char *demangled =
      abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
  if (status == 0) {
    name = demangled;
  }
  free(demangled);
#endif

  return name; // Return the demangled type name
}

/**
 * @class Blackboard
 * @brief A thread-safe storage for key-value pairs of varying types.
 *
 * The Blackboard class allows storing, retrieving, and managing
 * values associated with string keys in a thread-safe manner using
 * a recursive mutex.
 */
class Blackboard {
private:
  /// Mutex for thread safety.
  mutable std::recursive_mutex mutex;
  /// Storage for key-value pairs.
  std::unordered_map<std::string, std::shared_ptr<void>> values;
  /// Storage for type information for each key.
  TypeRegistry type_registry;
  /// Storage for key remappings.
  Remappings remappings;

  /** @brief Internal method that acquires the maped key. In the case the key is
   * not remaped, retruns the arg key.
   *  @param other The instance to copy from.
   */
  const std::string &remap(const std::string &key) const;

public:
  /**
   * @brief Shared pointer type for Blackboard.
   */
  YASMIN_SHARED_PTR_ALIAS(Blackboard)

  /**
   * @brief Default constructor for Blackboard.
   */
  Blackboard();

  /**
   * @brief Copy constructor for Blackboard.
   * @param other The instance to copy from.
   */
  Blackboard(const Blackboard &other);

  /**
   * @brief Set a value in the blackboard.
   * @tparam T The type of the value to store.
   * @param name The key to associate with the value.
   * @param value The value to store.
   */
  template <class T> void set(const std::string &name, T value) {

    YASMIN_LOG_DEBUG("Setting '%s' in the blackboard", name.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    // Apply remapping if exists
    std::string key = this->remap(name);

    // If the type is changing, remove the old entry first
    if (this->type_registry.find(key) != this->type_registry.end()) {
      this->values.erase(key);
      this->type_registry.erase(key);
    }

    // Insert value and type information if key does not exist
    if (!this->contains(key)) {
      this->values[key] = std::make_shared<T>(value);
      this->type_registry[key] = demangle_type(typeid(T).name());

    } else {
      // Check if the type is the same before updating
      if (this->type_registry.at(key) != demangle_type(typeid(T).name())) {
        this->values[key] = std::make_shared<T>(value);
        this->type_registry[key] = demangle_type(typeid(T).name());
        // Update the existing value
      } else {
        *(std::static_pointer_cast<T>(this->values.at(key))) = value;
      }
    }
  }

  /**
   * @brief Retrieve a value from the blackboard.
   * @tparam T The type of the value to retrieve.
   * @param name The key associated with the value.
   * @return The value associated with the specified key.
   * @throws std::runtime_error if the key does not exist.
   */
  template <class T> T get(const std::string &key) const {

    YASMIN_LOG_DEBUG("Getting '%s' from the blackboard", key.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    // Check if the key exists
    if (!this->contains(key)) {
      throw std::runtime_error("Element '" + key +
                               "' does not exist in the blackboard");
    }

    // Return the value casted to the requested type
    return *(std::static_pointer_cast<T>(this->values.at(this->remap(key))));
  }

  /**
   * @brief Remove a value from the blackboard.
   * @param key The key associated with the value to remove.
   */
  void remove(const std::string &key);

  /**
   * @brief Check if a key exists in the blackboard.
   * @param key The key to check.
   * @return True if the key exists, false otherwise.
   */
  bool contains(const std::string &key) const;

  /**
   * @brief Get the number of key-value pairs in the blackboard.
   * @return The size of the blackboard.
   */
  int size() const;

  /**
   * @brief Get the type of a value stored in the blackboard.
   * @param key The key associated with the value.
   * @return A string representation of the type.
   * @throws std::runtime_error if the key does not exist.
   */
  std::string get_type(const std::string &key) const;

  /**
   * @brief Convert the contents of the blackboard to a string.
   * @return A string representation of the blackboard.
   */
  std::string to_string() const;

  /**
   * @brief Set the remappings of the blackboard.
   * @param remappings The remappings to set.
   */
  void set_remappings(const Remappings &remappings);

  /**
   * @brief Get the remappings of the blackboard.
   * @return The remappings of the blackboard.
   */
  const Remappings &get_remappings() const noexcept;
};

} // namespace yasmin

#endif // YASMIN__BLACKBOARD_HPP
