// Copyright (C) 2024  Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS_YASMIN_NODE_HPP
#define YASMIN_ROS_YASMIN_NODE_HPP

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
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
  /**
   * @brief Default constructor. Initializes the node with a unique name.
   */
  explicit YasminNode();

  /**
   * @brief Deleted copy constructor to prevent copying of the singleton
   * instance.
   *
   * @param other Another instance of YasminNode (unused).
   */
  YasminNode(YasminNode &other) = delete;

  /**
   * @brief Deleted assignment operator to enforce singleton pattern.
   *
   * @param other Another instance of YasminNode (unused).
   */
  void operator=(const YasminNode &) = delete;

  /**
   * @brief Destructor. Cleans up resources.
   */
  ~YasminNode() {}

  /**
   * @brief Provides access to the singleton instance of YasminNode.
   *
   * This method ensures there is only one instance of YasminNode running.
   *
   * @return A shared pointer to the singleton instance of YasminNode.
   */
  static std::shared_ptr<YasminNode> get_instance() {
    static std::shared_ptr<YasminNode> instance =
        std::make_shared<YasminNode>();
    return instance;
  }

private:
  /// Executor for managing multiple threads.
  rclcpp::executors::MultiThreadedExecutor executor;
  /// Thread for spinning the node.
  std::unique_ptr<std::thread> spin_thread;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS_YASMIN_NODE_HPP
