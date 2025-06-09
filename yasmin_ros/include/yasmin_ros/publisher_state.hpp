// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__PUBLISHER_STATE_HPP
#define YASMIN_ROS__PUBLISHER_STATE_HPP

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief Template class to publish ROS 2 messages.
 *
 * This class provides functionality to publish to a ROS 2 topic of type
 * `MsgT` and create a custom messages.
 *
 * @tparam MsgT The message type of the topic to publish to.
 */
template <typename MsgT> class PublisherState : public yasmin::State {

  /// Function type for creating messages for topic.
  using CreateMessageHandler =
      std::function<MsgT(std::shared_ptr<yasmin::blackboard::Blackboard>)>;

public:
  /**
   * @brief Construct a new PublisherState with ROS 2 node and specific QoS.
   *
   * @param topic_name The name of the topic to monitor.
   * @param create_message_handler A callback handler to create messages.
   * @param qos Quality of Service settings for the topic.
   */
  PublisherState(std::string topic_name,
                 CreateMessageHandler create_message_handler,
                 rclcpp::QoS qos = 10,
                 rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
      : PublisherState(nullptr, topic_name, create_message_handler, qos,
                       callback_group) {}

  /**
   * @brief Construct a new PublisherState with ROS 2 node and specific QoS.
   *
   * @param node The ROS 2 node.
   * @param topic_name The name of the topic to monitor.
   * @param create_message_handler A callback handler to create messages.
   * @param qos Quality of Service settings for the topic.
   */
  PublisherState(const rclcpp::Node::SharedPtr &node, std::string topic_name,
                 CreateMessageHandler create_message_handler,
                 rclcpp::QoS qos = 10,
                 rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
      : State({}), topic_name(topic_name),
        create_message_handler(create_message_handler) {

    // set outcomes
    this->outcomes = {basic_outcomes::SUCCEED};

    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    // create subscriber
    rclcpp::PublisherOptions options;
    options.callback_group = callback_group;

    this->pub =
        this->node_->template create_publisher<MsgT>(topic_name, qos, options);

    if (this->create_message_handler == nullptr) {
      throw std::invalid_argument("create_message_handler is needed");
    }
  }

  /**
   * @brief Execute the publishing operation.
   *
   * @param blackboard A shared pointer to the blackboard for data storage.
   * @return A string outcome indicating the result of the monitoring operation.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    YASMIN_LOG_DEBUG("Publishing to topic '%s'", this->topic_name.c_str());
    MsgT msg = this->create_message_handler(blackboard);
    this->pub->publish(msg);
    return basic_outcomes::SUCCEED;
  }

private:
  rclcpp::Node::SharedPtr node_; /**< ROS 2 node pointer. */
  std::shared_ptr<rclcpp::Publisher<MsgT>>
      pub; /**< Publisher to the ROS 2 topic. */

  std::string topic_name; /**< Name of the topic to monitor. */
  CreateMessageHandler
      create_message_handler; /**< Callback handler to create messages. */
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__MONITOR_STATE_HPP
