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

#ifndef YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_
#define YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_

#include <string>

#include "yasmin/state.hpp"
#include "yasmin_ros/supported_interface_serialization.hpp"

namespace yasmin_ros {

/**
 * @brief A state for deserializing ROS 2 messages into C++ objects.
 */
class RosDeserializeCppState : public yasmin::State {
public:
  /**
   * @brief Construct a new RosDeserializeCppState.
   */
  RosDeserializeCppState();

  /** @brief Default destructor. */
  ~RosDeserializeCppState() override = default;

  /**
   * @brief Configure the state before execution.
   */
  void configure() override;

  /**
   * @brief Execute the deserialization operation.
   *
   * @param blackboard A shared pointer to the blackboard for data storage.
   * @return A string outcome indicating the result of the deserialization
   * operation.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief The type of the ROS 2 interface to deserialize.
  std::string interface_type_;
  /// @brief The handler for deserializing the ROS 2 message.
  const InterfaceSerializationHandler *handler_;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_
