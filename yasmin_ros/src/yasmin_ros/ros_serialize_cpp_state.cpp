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

#include "yasmin_ros/ros_serialize_cpp_state.hpp"

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

RosSerializeCppState::RosSerializeCppState()
    : yasmin::State({SUCCEED, TYPE_ERROR, ERROR}), handler_(nullptr) {
  this->set_description(
      "Serializes a C++ ROS interface instance from blackboard key 'input' "
      "into a byte array written to blackboard key 'output'.");
  this->set_outcome_description(SUCCEED,
                                "Serialization finished successfully.");
  this->set_outcome_description(
      TYPE_ERROR,
      "The value stored in 'input' does not match the configured interface.");
  this->set_outcome_description(
      ERROR, "Serialization failed because the input could not be processed.");
  this->add_input_key(
      "input",
      "ROS interface instance to serialize. The concrete type is selected "
      "through the interface_type parameter.");
  this->add_output_key(
      "output",
      "Serialized byte array produced from the ROS interface instance.");
  this->declare_parameter<std::string>(
      "interface_type",
      "ROS interface type string such as geometry_msgs/msg/Pose or "
      "std_srvs/srv/Trigger_Request.",
      std::string("geometry_msgs/msg/Pose"));
}

void RosSerializeCppState::configure() {
  this->interface_type_ = this->get_parameter<std::string>("interface_type");
  this->handler_ = &get_interface_serialization_handler(this->interface_type_);
}

std::string
RosSerializeCppState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    validate_blackboard_key_type(blackboard, "input",
                                 this->handler_->blackboard_type);

    const std::vector<uint8_t> serialized_data =
        this->handler_->serialize_from_blackboard(blackboard, "input");
    blackboard->set<std::vector<uint8_t>>("output", serialized_data);
    return SUCCEED;
  } catch (const std::runtime_error &e) {
    YASMIN_LOG_WARN("RosSerializeCppState type error for '%s': %s",
                    this->interface_type_.c_str(), e.what());
    return TYPE_ERROR;
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN("RosSerializeCppState failed for '%s': %s",
                    this->interface_type_.c_str(), e.what());
    return ERROR;
  }
}

} // namespace yasmin_ros

PLUGINLIB_EXPORT_CLASS(yasmin_ros::RosSerializeCppState, yasmin::State)
