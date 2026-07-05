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
