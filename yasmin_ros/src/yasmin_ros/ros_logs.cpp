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

#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

// Initialize logger ROS 2 node
rclcpp::Node *logger_node = nullptr;

/**
 * @brief Generalized logging function.
 *
 * @param level The log level as a yasmin::LogLevel (e.g., "ERROR", "WARN",
 * "INFO", "DEBUG").
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param args Additional arguments for the format string.
 */
void ros_log_message(yasmin::LogLevel level, const char *file,
                     const char *function, int line, const char *text) {

  std::ostringstream oss;
  oss << "[" << file << ":" << function << ":" << line << "] " << text;

  auto logger = logger_node != nullptr ? logger_node->get_logger()
                                       : rclcpp::get_logger("yasmin_ros");

  switch (level) {
  case yasmin::LogLevel::ERROR:
    RCLCPP_ERROR(logger, oss.str().c_str());
    break;
  case yasmin::LogLevel::WARN:
    RCLCPP_WARN(logger, oss.str().c_str());
    break;
  case yasmin::LogLevel::INFO:
    RCLCPP_INFO(logger, oss.str().c_str());
    break;
  case yasmin::LogLevel::DEBUG:
    RCLCPP_DEBUG(logger, oss.str().c_str());
    break;
  }
}

void set_ros_loggers(rclcpp::Node::SharedPtr node) {

  if (node == nullptr) {
    logger_node = YasminNode::get_instance().get();
  } else {
    logger_node = node.get();
  }

  yasmin::set_loggers(ros_log_message);
}

} // namespace yasmin_ros
