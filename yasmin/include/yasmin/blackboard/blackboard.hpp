// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_BLACKBOARD_HPP
#define YASMIN_BLACKBOARD_HPP

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

public:
  /** @brief Default constructor for Blackboard. */
  Blackboard();

  /** @brief Copy constructor for Blackboard.
   *  @param other The instance to copy from.
   */
  Blackboard(const Blackboard &other);

  /** @brief Destructor for Blackboard. */
  ~Blackboard();

  /**
   * @brief Retrieve a value from the blackboard.
   * @tparam T The type of the value to retrieve.
   * @param name The key associated with the value.
   * @return The value associated with the specified key.
   * @throws std::runtime_error if the key does not exist.
   */
  template <class T> T get(std::string name) {

    YASMIN_LOG_DEBUG("Getting '%s' from the blackboard", name.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    if (!this->contains(name)) {
      throw std::runtime_error("Element " + name +
                               " does not exist in the blackboard");
    }

    BlackboardValue<T> *b_value = (BlackboardValue<T> *)this->values.at(name);
    return b_value->get();
  }

  /**
   * @brief Set a value in the blackboard.
   * @tparam T The type of the value to store.
   * @param name The key to associate with the value.
   * @param value The value to store.
   */
  template <class T> void set(std::string name, T value) {

    YASMIN_LOG_DEBUG("Setting '%s' in the blackboard", name.c_str());

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    if (!this->contains(name)) {
      BlackboardValue<T> *b_value = new BlackboardValue<T>(value);
      this->values.insert({name, b_value});

    } else {
      ((BlackboardValue<T> *)this->values.at(name))->set(value);
    }
  }

  /**
   * @brief Remove a value from the blackboard.
   * @param name The key associated with the value to remove.
   */
  void remove(std::string name);

  /**
   * @brief Check if a key exists in the blackboard.
   * @param name The key to check.
   * @return True if the key exists, false otherwise.
   */
  bool contains(std::string name);

  /**
   * @brief Get the number of key-value pairs in the blackboard.
   * @return The size of the blackboard.
   */
  int size();

  /**
   * @brief Convert the contents of the blackboard to a string.
   * @return A string representation of the blackboard.
   */
  std::string to_string();
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN_BLACKBOARD_HPP
