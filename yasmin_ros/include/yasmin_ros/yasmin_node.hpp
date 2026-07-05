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

#ifndef YASMIN_ROS__YASMIN_NODE_HPP_
#define YASMIN_ROS__YASMIN_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {

/**
 * @class YasminNode
 * @brief A ROS 2 node for managing and handling YASMIN-based applications.
 *
 * YasminNode is a singleton class derived from rclcpp::Node and integrates
 * custom functionalities for executing specific tasks in a ROS 2 environment.
 */
class YasminNode : public rclcpp::Node {
public:
  /** @brief Shared pointer type for YasminNode. */
  using SharedPtr = std::shared_ptr<YasminNode>;

protected:
  /**
   * @brief Default constructor. Initializes the node with a unique name.
   */
  explicit YasminNode();

public:
  /** @brief Deleted copy constructor (singleton). */
  YasminNode(YasminNode &other) = delete;

  /**
   * @brief Destructor. Cleans up resources.
   */
  ~YasminNode();

  /** @brief Deleted copy assignment (singleton). */
  void operator=(const YasminNode &) = delete;

  /**
   * @brief Provides access to the singleton instance of YasminNode.
   *
   * This method ensures there is only one instance of YasminNode running.
   *
   * @return A shared pointer to the singleton instance of YasminNode.
   */
  static YasminNode::SharedPtr get_instance();

  /**
   * @brief Destroy the singleton instance if it exists.
   */
  static void destroy_instance();

private:
  /**
   * @brief Stop the executor thread and remove the node from the executor.
   */
  void stop_executor();

  /// @brief Executor for managing multiple threads.
#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 1, 1) // Jazzy, Kilted and Rolling
  rclcpp::experimental::executors::EventsExecutor executor;
#else // Humble, Iron and Jazzy
  rclcpp::executors::MultiThreadedExecutor executor;
#endif
#else // Foxy and Galactic
  rclcpp::executors::MultiThreadedExecutor executor;
#endif
  /// @brief Thread for spinning the node.
  std::unique_ptr<std::thread> spin_thread;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__YASMIN_NODE_HPP_
