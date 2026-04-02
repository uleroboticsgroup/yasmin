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

#include "yasmin_ros/ros_deserialize_cpp_state.hpp"

#include <exception>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "yasmin/logs.hpp"

namespace yasmin_ros {
namespace {

const std::string SUCCEED = "succeed";
const std::string TYPE_ERROR = "type_error";
const std::string ERROR = "error";

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
