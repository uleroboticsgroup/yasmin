// Copyright (C) 2026 Maik Knof
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
  std::function<std::vector<uint8_t>(yasmin::Blackboard::SharedPtr,
                                     const std::string &)>
      serialize_from_blackboard;
  std::function<void(yasmin::Blackboard::SharedPtr, const std::string &,
                     const std::vector<uint8_t> &)>
      deserialize_to_blackboard;
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
