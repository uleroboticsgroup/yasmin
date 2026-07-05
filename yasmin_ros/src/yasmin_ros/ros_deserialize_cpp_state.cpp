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

#include "yasmin_ros/ros_deserialize_cpp_state.hpp"

#include <exception>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/logs.hpp"

namespace yasmin_ros {
namespace {

const std::string SUCCEED = "succeed";
const std::string TYPE_ERROR = "type_error";
const std::string ERROR = "error";
const std::string SERIALIZED_BYTES_TYPE =
    yasmin::demangle_type(typeid(std::vector<uint8_t>).name());

void validate_blackboard_key_type(
    const yasmin::Blackboard::SharedPtr &blackboard, const std::string &key,
    const std::string &expected_type) {
  if (!blackboard->contains(key)) {
    throw std::runtime_error("Element '" + key +
                             "' does not exist in the blackboard");
  }

  const std::string actual_type = blackboard->get_type(key);
  if (actual_type != expected_type) {
    throw std::runtime_error("Element '" + key + "' has type '" + actual_type +
                             "' but expected '" + expected_type + "'");
  }
}

} // namespace

RosDeserializeCppState::RosDeserializeCppState()
    : yasmin::State({SUCCEED, TYPE_ERROR, ERROR}), handler_(nullptr) {
  this->set_description(
      "Deserializes a byte array from blackboard key 'input' into a C++ ROS "
      "interface instance written to blackboard key 'output'.");
  this->set_outcome_description(SUCCEED,
                                "Deserialization finished successfully.");
  this->set_outcome_description(
      TYPE_ERROR,
      "The value stored in 'input' is not a byte array or the target type "
      "does not match.");
  this->set_outcome_description(
      ERROR, "Deserialization failed because the serialized data could not be "
             "decoded.");
  this->add_input_key(
      "input", "Serialized byte array that should be converted back into a ROS "
               "interface instance.");
  this->add_output_key(
      "output", "Deserialized ROS interface instance in native C++ form.");
  this->declare_parameter<std::string>(
      "interface_type",
      "ROS interface type string such as geometry_msgs/msg/Pose or "
      "example_interfaces/action/Fibonacci_Result.",
      std::string("geometry_msgs/msg/Pose"));
}

void RosDeserializeCppState::configure() {
  this->interface_type_ = this->get_parameter<std::string>("interface_type");
  this->handler_ = &get_interface_serialization_handler(this->interface_type_);
}

std::string
RosDeserializeCppState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    validate_blackboard_key_type(blackboard, "input", SERIALIZED_BYTES_TYPE);

    const std::vector<uint8_t> serialized_data =
        blackboard->get<std::vector<uint8_t>>("input");
    this->handler_->deserialize_to_blackboard(blackboard, "output",
                                              serialized_data);
    return SUCCEED;
  } catch (const std::runtime_error &e) {
    YASMIN_LOG_WARN("RosDeserializeCppState type error for '%s': %s",
                    this->interface_type_.c_str(), e.what());
    return TYPE_ERROR;
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN("RosDeserializeCppState failed for '%s': %s",
                    this->interface_type_.c_str(), e.what());
    return ERROR;
  }
}

} // namespace yasmin_ros

PLUGINLIB_EXPORT_CLASS(yasmin_ros::RosDeserializeCppState, yasmin::State)
