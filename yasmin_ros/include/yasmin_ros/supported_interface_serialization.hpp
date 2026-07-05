// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN_ROS__SUPPORTED_INTERFACE_SERIALIZATION_HPP_
#define YASMIN_ROS__SUPPORTED_INTERFACE_SERIALIZATION_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "yasmin/blackboard.hpp"

namespace yasmin_ros {

/**
 * @brief A structure to hold the serialization handlers for a specific
 * interface type.
 */
struct InterfaceSerializationHandler {
  /// @brief Callback to serialize a ROS interface from blackboard values
  std::function<std::vector<uint8_t>(yasmin::Blackboard::SharedPtr,
                                     const std::string &)>
      serialize_from_blackboard;
  /// @brief Callback to deserialize a ROS interface into blackboard values
  std::function<void(yasmin::Blackboard::SharedPtr, const std::string &,
                     const std::vector<uint8_t> &)>
      deserialize_to_blackboard;
  /// @brief The blackboard type string for this interface
  std::string blackboard_type;
};

/**
 * @brief Get the map of supported interface serialization handlers.
 *
 * @return A reference to the unordered map containing interface types and their
 * corresponding serialization handlers.
 */
const std::unordered_map<std::string, InterfaceSerializationHandler> &
get_supported_interface_serialization_handlers();

/**
 * @brief Check if a given interface type is supported for serialization.
 *
 * @param interface_type The type of the interface to check.
 * @return true if the interface type is supported, false otherwise.
 */
bool is_supported_interface_type(const std::string &interface_type);

/**
 * @brief Get the serialization handler for a specific interface type.
 *
 * @param interface_type The type of the interface to get the handler for.
 * @return A reference to the InterfaceSerializationHandler for the specified
 * interface type.
 * @throws std::invalid_argument if the interface type is not supported.
 */
const InterfaceSerializationHandler &
get_interface_serialization_handler(const std::string &interface_type);

/**
 * @brief Get a list of supported interface types for serialization.
 *
 * @return A vector of strings containing the supported interface types.
 */
std::vector<std::string> get_supported_interface_types();

} // namespace yasmin_ros

#endif // YASMIN_ROS__SUPPORTED_INTERFACE_SERIALIZATION_HPP_
