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

#ifndef YASMIN__BLACKBOARD_KEY_INFO_HPP_
#define YASMIN__BLACKBOARD_KEY_INFO_HPP_

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>
#include <unordered_map>

#include "yasmin/blackboard.hpp"

namespace yasmin {

/**
 * @struct BlackboardKeyInfo
 * @brief Information about a blackboard key including name and optional default
 * value.
 */
struct BlackboardKeyInfo {
  /// The name of the key
  std::string name;
  /// Human-readable description of the key
  std::string description;
  /// Whether this key has a default value
  bool has_default{false};
  /// The default value stored as a shared pointer to void (similar to
  /// Blackboard)
  std::shared_ptr<void> default_value{};
  /// The type name of the default value
  std::string default_value_type{};
  /// Function to inject the default value into a blackboard at the given key
  std::function<void(Blackboard &, const std::string &)> inject_default{};

  /**
   * @brief Default constructor.
   */
  BlackboardKeyInfo() = default;

  /**
   * @brief Constructor with key name only.
   * @param key_name The name of the blackboard key.
   */
  explicit BlackboardKeyInfo(std::string key_name)
      : name(std::move(key_name)) {}

  /**
   * @brief Constructor with key name and description.
   * @param key_name The name of the blackboard key.
   * @param key_description Human-readable description of the blackboard key.
   */
  BlackboardKeyInfo(std::string key_name, std::string key_description)
      : name(std::move(key_name)), description(std::move(key_description)) {}

  /**
   * @brief Constructor with key name, default value and description.
   * @tparam T The type of the default value.
   * @param key_name The name of the blackboard key.
   * @param key_description Human-readable description of the blackboard key.
   * @param value The default value.
   */
  template <typename T>
  BlackboardKeyInfo(std::string key_name, std::string key_description, T value)
      : name(std::move(key_name)), description(std::move(key_description)),
        has_default(true), default_value(std::make_shared<std::decay_t<T>>(
                               std::forward<T>(value))),
        default_value_type(demangle_type(typeid(std::decay_t<T>).name())) {
    using ValueType = std::decay_t<T>;
    auto stored = std::static_pointer_cast<ValueType>(this->default_value);
    inject_default = [stored](Blackboard &bb, const std::string &key) {
      bb.set<ValueType>(key, *stored);
    };
  }

  /**
   * @brief Constructor with key name, default value and description for const
   * char *.
   * @param key_name The name of the blackboard key.
   * @param key_description Human-readable description of the blackboard key.
   * @param value The default value.
   */
  BlackboardKeyInfo(std::string key_name, std::string key_description,
                    const char *value)
      : name(std::move(key_name)), description(std::move(key_description)),
        has_default(true), default_value(std::make_shared<std::string>(value)),
        default_value_type(demangle_type(typeid(std::string).name())) {
    auto stored = std::static_pointer_cast<std::string>(this->default_value);
    inject_default = [stored](Blackboard &bb, const std::string &key) {
      bb.set<std::string>(key, *stored);
    };
  }

  /**
   * @brief Retrieve the default value as the given type.
   * @tparam T The type to cast the default value to.
   * @return The default value.
   */
  template <typename T> T get_default_value() const {
    return *std::static_pointer_cast<T>(this->default_value);
  }
};

/**
 * @struct StateMetadata
 * @brief Metadata for a state including description and blackboard key
 * information.
 */
struct StateMetadata {
  /// Description of the state
  std::string description;
  /// Human-readable descriptions for outcomes, indexed by outcome name
  std::unordered_map<std::string, std::string> outcome_descriptions;
  /// Information about input keys required by this state
  std::vector<BlackboardKeyInfo> input_keys;
  /// Information about output keys produced by this state
  std::vector<BlackboardKeyInfo> output_keys;
};

} // namespace yasmin

#endif // YASMIN__BLACKBOARD_KEY_INFO_HPP_
