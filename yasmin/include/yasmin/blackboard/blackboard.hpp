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

#ifndef YASMIN__BLACKBOARD__BLACKBOARD_HPP
#define YASMIN__BLACKBOARD__BLACKBOARD_HPP

#include <exception>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>

#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_interface.hpp"
#include "yasmin/logs.hpp"

namespace yasmin {
namespace blackboard {

/**
 * @class Blackboard
 * @brief A thread-safe storage for key-value pairs of varying types.
 *
 * The Blackboard class allows storing, retrieving, and managing
 * values associated with string keys in a thread-safe manner using
 * a recursive mutex. Values are stored as pointers to
 * BlackboardValueInterface instances.
 */
class Blackboard {
private:
  /// Mutex for thread safety.
  std::recursive_mutex mutex;
  /// Storage for key-value pairs.
  std::map<std::string, BlackboardValueInterface *> values;
  /// Storage for type information for each key.
  std::map<std::string, std::string> type_registry;
  /// Storage for key remappings.
  std::map<std::string, std::string> remappings;

  /** @brief Internal method that acquires the maped key. In the case the key is
   * not remaped, retruns the arg key.
   *  @param other The instance to copy from.
   */
  const std::string &remap(const std::string &key);

public:
  /** @brief Default constructor for Blackboard. */
  Blackboard();

  /** @brief Copy constructor for Blackboard.
   *  @param other The instance to copy from.
   */
  Blackboard(const Blackboard &other);

  /** @brief Virtual destructor for Blackboard. */
  virtual ~Blackboard();

  /**
   * @brief Set a value in the blackboard.
   * @tparam T The type of the value to store.
   * @param name The key to associate with the value.
   * @param value The value to store.
   */
  template <class T> void set(const std::string &name, T value) {

    YASMIN_LOG_DEBUG("Setting '%s' in the blackboard", name.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    BlackboardValue<T> *b_value = new BlackboardValue<T>(value);

    // Apply remapping if exists
    std::string key = this->remap(name);

    // If the type is changing, remove the old entry first
    if (this->type_registry.find(key) != this->type_registry.end()) {
      this->values.erase(key);
      this->type_registry.erase(key);
    }

    // Insert or update the value
    if (!this->contains(key)) {
      this->values.insert({key, b_value});
      this->type_registry.insert({key, b_value->get_type()});

    } else {
      b_value = (BlackboardValue<T> *)this->values.at(key);
      b_value->set(value);
      this->type_registry[key] = b_value->get_type();
    }
  }

  /**
   * @brief Retrieve a value from the blackboard.
   * @tparam T The type of the value to retrieve.
   * @param name The key associated with the value.
   * @return The value associated with the specified key.
   * @throws std::runtime_error if the key does not exist.
   */
  template <class T> T get(const std::string &key) {

    YASMIN_LOG_DEBUG("Getting '%s' from the blackboard", key.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    if (!this->contains(key)) {
      throw std::runtime_error("Element '" + key +
                               "' does not exist in the blackboard");
    }

    BlackboardValue<T> *b_value =
        (BlackboardValue<T> *)this->values.at(this->remap(key));
    return b_value->get();
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
  bool contains(const std::string &key);

  /**
   * @brief Get the number of key-value pairs in the blackboard.
   * @return The size of the blackboard.
   */
  int size();

  /**
   * @brief Get the type of a value stored in the blackboard.
   * @param key The key associated with the value.
   * @return A string representation of the type.
   * @throws std::runtime_error if the key does not exist.
   */
  std::string get_type(const std::string &key);

  /**
   * @brief Convert the contents of the blackboard to a string.
   * @return A string representation of the blackboard.
   */
  std::string to_string();

  /**
   * @brief Set the remappings of the blackboard.
   * @param remappings The remappings to set.
   */
  void set_remappings(const std::map<std::string, std::string> &remappings);

  /**
   * @brief Get the remappings of the blackboard.
   * @return The remappings of the blackboard.
   */
  const std::map<std::string, std::string> &get_remappings();
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN__BLACKBOARD_HPP
