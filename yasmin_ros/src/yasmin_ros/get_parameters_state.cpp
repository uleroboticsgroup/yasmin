// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include "rclcpp/rclcpp.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "yasmin_ros/get_parameters_state.hpp"

using namespace yasmin_ros;

GetParametersState::GetParametersState(
    const std::map<std::string, std::any> &parameters,
    rclcpp::Node::SharedPtr node)
    : yasmin::State({basic_outcomes::SUCCEED, basic_outcomes::ABORT}),
      parameters_(parameters) {

  if (node == nullptr) {
    this->node_ = YasminNode::get_instance();
  } else {
    this->node_ = node;
  }
}

std::string GetParametersState::execute(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  for (const auto &param : this->parameters_) {
    const std::string &param_name = param.first;
    const std::any &default_value = param.second;

    if (!this->node_->has_parameter(param_name)) {

      if (default_value.type() == typeid(bool)) {
        this->node_->declare_parameter(param_name,
                                       std::any_cast<bool>(default_value));
      } else if (default_value.type() == typeid(int)) {
        this->node_->declare_parameter(param_name,
                                       std::any_cast<int>(default_value));
      } else if (default_value.type() == typeid(double)) {
        this->node_->declare_parameter(param_name,
                                       std::any_cast<double>(default_value));
      } else if (default_value.type() == typeid(std::string)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::string>(default_value));
      } else if (default_value.type() == typeid(std::vector<bool>)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::vector<bool>>(default_value));
      } else if (default_value.type() == typeid(std::vector<int>)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::vector<int64_t>>(default_value));
      } else if (default_value.type() == typeid(std::vector<double>)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::vector<double>>(default_value));
      } else if (default_value.type() == typeid(std::vector<std::string>)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::vector<std::string>>(default_value));
      } else if (default_value.type() == typeid(std::vector<uint8_t>)) {
        this->node_->declare_parameter(
            param_name, std::any_cast<std::vector<uint8_t>>(default_value));
      } else {
        YASMIN_LOG_ERROR("Unsupported default type for parameter: '%s'",
                         param_name.c_str());
        return basic_outcomes::ABORT;
      }
    }

    YASMIN_LOG_INFO("Retrieving parameter '%s'", param_name.c_str());

    auto parameter = this->node_->get_parameter(param_name);
    auto type = this->node_->get_parameter_types({param_name});

    switch (type[0]) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      blackboard->set<bool>(param_name, parameter.as_bool());
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      blackboard->set<int64_t>(param_name, parameter.as_int());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      blackboard->set<double>(param_name, parameter.as_double());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      blackboard->set<std::string>(param_name, parameter.as_string());
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      blackboard->set<std::vector<bool>>(param_name, parameter.as_bool_array());
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      blackboard->set<std::vector<int64_t>>(param_name,
                                            parameter.as_integer_array());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      blackboard->set<std::vector<double>>(param_name,
                                           parameter.as_double_array());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      blackboard->set<std::vector<std::string>>(param_name,
                                                parameter.as_string_array());
      break;
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      blackboard->set<std::vector<uint8_t>>(param_name,
                                            parameter.as_byte_array());
      break;
    default:
      YASMIN_LOG_ERROR("Unsupported parameter type for '%s'",
                       param_name.c_str());
      return basic_outcomes::ABORT;
    }
  }

  return basic_outcomes::SUCCEED;
}
