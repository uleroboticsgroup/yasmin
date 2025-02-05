// Copyright (C) 2024 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <cstdarg>
#include <rclcpp/rclcpp.hpp>
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
void log_message(yasmin::LogLevel level, const char *file, const char *function,
                 int line, const char *text) {
  switch (level) {
  case yasmin::LogLevel::ERROR:
    RCLCPP_ERROR(logger_node->get_logger(), "[%s:%s:%d] %s", file, function,
                 line, text);
    break;
  case yasmin::LogLevel::WARN:
    RCLCPP_WARN(logger_node->get_logger(), "[%s:%s:%d] %s", file, function,
                line, text);
    break;
  case yasmin::LogLevel::INFO:
    RCLCPP_INFO(logger_node->get_logger(), "[%s:%s:%d] %s", file, function,
                line, text);
    break;
  case yasmin::LogLevel::DEBUG:
    RCLCPP_DEBUG(logger_node->get_logger(), "[%s:%s:%d] %s", file, function,
                 line, text);
    break;
  }
}

/**
 * @brief Variadic template function to log messages at different levels.
 *
 * This function wraps log_message and allows logging messages with different
 * log levels while reducing redundant code. It provides a consistent logging
 * format across all levels.
 *
 * @tparam LEVEL The log level LogLevel (e.g., 0 -> "ERROR", 1 -> "WARN", 2 ->
 * "INFO", 3 -> "DEBUG").
 * @param log_message Function to create the logs
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param ... Additional arguments for the format string.
 */
template <yasmin::LogLevel LEVEL>
void log_helper(const char *file, const char *function, int line,
                const char *text, ...) {
  va_list args;
  va_start(args, text);

  // Calculate the required buffer size
  int size = vsnprintf(nullptr, 0, text, args) + 1;
  va_end(args);

  std::string buffer(size, '\0');
  va_start(args, text);
  vsnprintf(&buffer[0], buffer.size(), text, args);

  va_end(args);

  log_message(LEVEL, file, function, line, buffer.c_str());
}

void set_ros_loggers(rclcpp::Node::SharedPtr node) {
  if (node == nullptr) {
    logger_node = YasminNode::get_instance().get();
  } else {
    logger_node = node.get();
  }
  yasmin::set_loggers(
      log_helper<yasmin::LogLevel::ERROR>, log_helper<yasmin::LogLevel::WARN>,
      log_helper<yasmin::LogLevel::INFO>, log_helper<yasmin::LogLevel::DEBUG>);
}

} // namespace yasmin_ros