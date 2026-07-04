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

#ifndef YASMIN_ROS__INTERFACE_SERIALIZATION_HPP_
#define YASMIN_ROS__INTERFACE_SERIALIZATION_HPP_

#include <cstring>
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
