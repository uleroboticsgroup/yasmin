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

#include "yasmin_ros/supported_interface_serialization.hpp"

#include <stdexcept>

#include <example_interfaces/action/fibonacci.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin_ros/interface_serialization.hpp"

namespace yasmin_ros {
namespace {

template <typename InterfaceT> InterfaceSerializationHandler make_handler() {
  return InterfaceSerializationHandler{
      [](yasmin::Blackboard::SharedPtr blackboard, const std::string &key) {
        return serialize_interface<InterfaceT>(
            blackboard->get<InterfaceT>(key));
      },
      [](yasmin::Blackboard::SharedPtr blackboard, const std::string &key,
         const std::vector<uint8_t> &serialized_data) {
        blackboard->set<InterfaceT>(
            key, deserialize_interface<InterfaceT>(serialized_data));
      },
      yasmin::demangle_type(typeid(InterfaceT).name())};
}

const std::unordered_map<std::string, InterfaceSerializationHandler>
    supported_handlers = {
        {"geometry_msgs/msg/Pose", make_handler<geometry_msgs::msg::Pose>()},
        {"std_msgs/msg/String", make_handler<std_msgs::msg::String>()},
        {"std_srvs/srv/Trigger_Request",
         make_handler<std_srvs::srv::Trigger::Request>()},
        {"std_srvs/srv/Trigger_Response",
         make_handler<std_srvs::srv::Trigger::Response>()},
        {"example_interfaces/action/Fibonacci_Goal",
         make_handler<example_interfaces::action::Fibonacci::Goal>()},
        {"example_interfaces/action/Fibonacci_Result",
         make_handler<example_interfaces::action::Fibonacci::Result>()},
        {"example_interfaces/action/Fibonacci_Feedback",
         make_handler<example_interfaces::action::Fibonacci::Feedback>()},
};

} // namespace

const std::unordered_map<std::string, InterfaceSerializationHandler> &
get_supported_interface_serialization_handlers() {
  return supported_handlers;
}

bool is_supported_interface_type(const std::string &interface_type) {
  return supported_handlers.find(interface_type) != supported_handlers.end();
}

const InterfaceSerializationHandler &
get_interface_serialization_handler(const std::string &interface_type) {
  auto it = supported_handlers.find(interface_type);
  if (it == supported_handlers.end()) {
    throw std::invalid_argument("Unsupported interface type '" +
                                interface_type + "'.");
  }
  return it->second;
}

std::vector<std::string> get_supported_interface_types() {
  std::vector<std::string> interface_types;
  interface_types.reserve(supported_handlers.size());

  for (const auto &entry : supported_handlers) {
    interface_types.push_back(entry.first);
  }

  return interface_types;
}

} // namespace yasmin_ros
