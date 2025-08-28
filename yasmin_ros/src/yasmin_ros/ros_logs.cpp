// Copyright (C) 2024 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <cstdarg>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

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

  switch (level) {
  case yasmin::LogLevel::ERROR:
    RCLCPP_ERROR(logger_node->get_logger(), oss.str().c_str());
    break;
  case yasmin::LogLevel::WARN:
    RCLCPP_WARN(logger_node->get_logger(), oss.str().c_str());
    break;
  case yasmin::LogLevel::INFO:
    RCLCPP_INFO(logger_node->get_logger(), oss.str().c_str());
    break;
  case yasmin::LogLevel::DEBUG:
    RCLCPP_DEBUG(logger_node->get_logger(), oss.str().c_str());
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