// Copyright (C) 2024  Miguel Ángel González Santamarta
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

/**
 * @brief Logs an error message with formatted text to the ROS logger.
 *
 * @param file The name of the source file where the log call is made.
 * @param function The name of the function where the log call is made.
 * @param line The line number in the source file where the log call is made.
 * @param text The format string for the message to be logged, followed by any
 * format arguments.
 */
void ros_log_error(const char *file, const char *function, int line,
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

  // Log the formatted error message
  RCLCPP_ERROR(YasminNode::get_instance()->get_logger(), "[%s:%s:%d] %s", file,
               function, line, buffer.c_str());
}

/**
 * @brief Logs a warning message with formatted text to the ROS logger.
 *
 * @param file The name of the source file where the log call is made.
 * @param function The name of the function where the log call is made.
 * @param line The line number in the source file where the log call is made.
 * @param text The format string for the message to be logged, followed by any
 * format arguments.
 */
void ros_log_warn(const char *file, const char *function, int line,
                  const char *text, ...) {
  va_list args;
  va_start(args, text);

  int size = vsnprintf(nullptr, 0, text, args) + 1;
  va_end(args);

  std::string buffer(size, '\0');

  va_start(args, text);
  vsnprintf(&buffer[0], buffer.size(), text, args);
  va_end(args);

  // Log the formatted warning message
  RCLCPP_WARN(YasminNode::get_instance()->get_logger(), "[%s:%s:%d] %s", file,
              function, line, buffer.c_str());
}

/**
 * @brief Logs an informational message with formatted text to the ROS logger.
 *
 * @param file The name of the source file where the log call is made.
 * @param function The name of the function where the log call is made.
 * @param line The line number in the source file where the log call is made.
 * @param text The format string for the message to be logged, followed by any
 * format arguments.
 */
void ros_log_info(const char *file, const char *function, int line,
                  const char *text, ...) {
  va_list args;
  va_start(args, text);

  int size = vsnprintf(nullptr, 0, text, args) + 1;
  va_end(args);

  std::string buffer(size, '\0');

  va_start(args, text);
  vsnprintf(&buffer[0], buffer.size(), text, args);
  va_end(args);

  // Log the formatted informational message
  RCLCPP_INFO(YasminNode::get_instance()->get_logger(), "[%s:%s:%d] %s", file,
              function, line, buffer.c_str());
}

/**
 * @brief Logs a debug message with formatted text to the ROS logger.
 *
 * @param file The name of the source file where the log call is made.
 * @param function The name of the function where the log call is made.
 * @param line The line number in the source file where the log call is made.
 * @param text The format string for the message to be logged, followed by any
 * format arguments.
 */
void ros_log_debug(const char *file, const char *function, int line,
                   const char *text, ...) {
  va_list args;
  va_start(args, text);

  int size = vsnprintf(nullptr, 0, text, args) + 1;
  va_end(args);

  std::string buffer(size, '\0');

  va_start(args, text);
  vsnprintf(&buffer[0], buffer.size(), text, args);
  va_end(args);

  // Log the formatted debug message
  RCLCPP_DEBUG(YasminNode::get_instance()->get_logger(), "[%s:%s:%d] %s", file,
               function, line, buffer.c_str());
}

void set_ros_loggers() {
  yasmin::set_loggers(ros_log_error, ros_log_warn, ros_log_info, ros_log_debug);
}

} // namespace yasmin_ros
