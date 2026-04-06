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

#ifndef YASMIN_ROS__INTERFACE_SERIALIZATION_HPP_
#define YASMIN_ROS__INTERFACE_SERIALIZATION_HPP_

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace yasmin_ros {

/**
 * @brief Serialize a ROS 2 interface instance to a byte array.
 *
 * @tparam InterfaceT ROS 2 message, service request/response, or action
 * goal/result/feedback type.
 * @param interface_value The interface instance to serialize.
 * @return Serialized byte array.
 */
template <typename InterfaceT>
std::vector<uint8_t> serialize_interface(const InterfaceT &interface_value) {
  rclcpp::Serialization<InterfaceT> serializer;
  rclcpp::SerializedMessage serialized_message;
  serializer.serialize_message(&interface_value, &serialized_message);

  const auto &rcl_serialized_message =
      serialized_message.get_rcl_serialized_message();

  return std::vector<uint8_t>(rcl_serialized_message.buffer,
                              rcl_serialized_message.buffer +
                                  rcl_serialized_message.buffer_length);
}

/**
 * @brief Deserialize a ROS 2 interface instance from a byte array.
 *
 * @tparam InterfaceT ROS 2 message, service request/response, or action
 * goal/result/feedback type.
 * @param serialized_data Serialized byte array.
 * @return Deserialized interface instance.
 */
template <typename InterfaceT>
InterfaceT deserialize_interface(const std::vector<uint8_t> &serialized_data) {
  rclcpp::Serialization<InterfaceT> serializer;
  rclcpp::SerializedMessage serialized_message(serialized_data.size());

  auto &rcl_serialized_message =
      serialized_message.get_rcl_serialized_message();

  if (!serialized_data.empty()) {
    std::memcpy(rcl_serialized_message.buffer, serialized_data.data(),
                serialized_data.size());
  }

  rcl_serialized_message.buffer_length = serialized_data.size();

  InterfaceT interface_value;
  serializer.deserialize_message(&serialized_message, &interface_value);

  return interface_value;
}

} // namespace yasmin_ros

#endif // YASMIN_ROS__INTERFACE_SERIALIZATION_HPP_
