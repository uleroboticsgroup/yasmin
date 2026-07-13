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

#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

std::shared_ptr<rclcpp::Node> logger_node;
std::mutex logger_node_mutex;

void ros_log_message(yasmin::LogLevel level, const char *file,
                     const char *function, int line, const char *text) {

  std::shared_ptr<rclcpp::Node> local_node;
  {
    std::lock_guard<std::mutex> lock(logger_node_mutex);
    local_node = logger_node;
  }

  auto logger =
      local_node ? local_node->get_logger() : rclcpp::get_logger("yasmin_ros");

  auto format_msg = [&]() {
    std::ostringstream oss;
    oss << "[" << file << ":" << function << ":" << line << "] " << text;
    return oss.str();
  };

  switch (level) {
  case yasmin::LogLevel::ERROR:
    RCLCPP_ERROR(logger, "%s", format_msg().c_str());
    break;
  case yasmin::LogLevel::WARN:
    RCLCPP_WARN(logger, "%s", format_msg().c_str());
    break;
  case yasmin::LogLevel::INFO:
    RCLCPP_INFO(logger, "%s", format_msg().c_str());
    break;
  case yasmin::LogLevel::DEBUG:
    RCLCPP_DEBUG(logger, "%s", format_msg().c_str());
    break;
  }
}

void set_ros_loggers(rclcpp::Node::SharedPtr node) {

  {
    std::lock_guard<std::mutex> lock(logger_node_mutex);

    if (node == nullptr) {
      logger_node = YasminNode::get_instance();
    } else {
      logger_node = node;
    }
  }

  yasmin::set_loggers(ros_log_message);
}

} // namespace yasmin_ros
