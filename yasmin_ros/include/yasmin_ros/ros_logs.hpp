// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__LOGS_HPP_
#define YASMIN_ROS__LOGS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace yasmin_ros {

/// @brief Node used in the YASMIN ROS 2 logs
extern rclcpp::Node *logger_node;

/**
 * @brief Sets the logging functions for ROS, linking ROS 2 log levels to YASMIN
 * loggers.
 *
 * This function configures YASMIN to use ROS 2 logging mechanisms for error,
 * warning, info, and debug levels, ensuring messages from YASMIN are routed
 * through ROS 2 loggers.
 *
 * @param node ROS 2 node to use as logger node. If null, use YasminNode.
 *
 * @note This function should be called once to set up the loggers before any
 * logging occurs within YASMIN components.
 */
void set_ros_loggers(rclcpp::Node::SharedPtr node = nullptr);

} // namespace yasmin_ros

#endif // YASMIN_ROS__LOGS_HPP_
