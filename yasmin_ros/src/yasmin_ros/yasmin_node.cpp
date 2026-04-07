// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_ros/ros_logs.hpp"

#include <random>
#include <string>

using namespace yasmin_ros;

namespace {
YasminNode::SharedPtr &get_yasmin_node_instance() {
  static YasminNode::SharedPtr instance;
  return instance;
}

std::mutex &get_yasmin_node_instance_mutex() {
  static std::mutex mutex;
  return mutex;
}
} // namespace

/**
 * @brief Generates a unique UUID as a string.
 *
 * This function uses random numbers to generate a 16-character hexadecimal
 * UUID.
 *
 * @return A string containing a 16-character hexadecimal UUID.
 */
inline std::string generateUUID() {
  static constexpr char hex_digits[] = "0123456789abcdef";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);

  std::string result;
  result.reserve(16);
  for (int i = 0; i < 16; ++i) {
    result += hex_digits[dis(gen)];
  }
  return result;
}

YasminNode::SharedPtr YasminNode::get_instance() {
  std::lock_guard<std::mutex> lock(get_yasmin_node_instance_mutex());

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto &instance = get_yasmin_node_instance();
  if (instance == nullptr) {
    instance = SharedPtr(new YasminNode());
  }

  return instance;
}

void YasminNode::destroy_instance() {
  std::lock_guard<std::mutex> lock(get_yasmin_node_instance_mutex());
  auto &instance = get_yasmin_node_instance();
  if (logger_node == instance.get()) {
    logger_node = nullptr;
  }
  instance.reset();
}

YasminNode::YasminNode() : rclcpp::Node("yasmin_" + generateUUID() + "_node") {
  // Add this node's base interface to the executor for multi-threaded
  // execution.
  this->executor.add_node(this->get_node_base_interface());

  // Initialize the spin thread to run the executor asynchronously.
  this->spin_thread =
      std::make_unique<std::thread>(&rclcpp::Executor::spin, &this->executor);
}

YasminNode::~YasminNode() {
  if (logger_node == this) {
    logger_node = nullptr;
  }
  this->stop_executor();
}

void YasminNode::stop_executor() {
  if (this->spin_thread == nullptr) {
    return;
  }

  this->executor.cancel();
  this->executor.remove_node(this->get_node_base_interface());

  if (this->spin_thread->joinable()) {
    this->spin_thread->join();
  }

  this->spin_thread.reset();
}
