// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS_MONITOR_STATE_HPP
#define YASMIN_ROS_MONITOR_STATE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using std::placeholders::_1;

namespace yasmin_ros {

/**
 * @brief Template class to monitor a ROS 2 topic and process incoming messages.
 *
 * This class provides functionality to subscribe to a ROS 2 topic of type
 * `MsgT`, execute a custom monitoring handler, and return specific outcomes
 * based on the messages received.
 *
 * @tparam MsgT The message type of the topic to subscribe to.
 */
template <typename MsgT> class MonitorState : public yasmin::State {

  /// Function type for handling messages from topic.
  using MonitorHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, std::shared_ptr<MsgT>)>;

public:
  /**
   * @brief Construct a new MonitorState with specific QoS and message queue
   * settings.
   *
   * @param topic_name The name of the topic to monitor.
   * @param outcomes A set of possible outcomes for this state.
   * @param monitor_handler A callback handler to process incoming messages.
   * @param qos Quality of Service settings for the topic.
   * @param msg_queue The maximum number of messages to queue.
   */
  MonitorState(std::string topic_name, std::set<std::string> outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos, int msg_queue)
      : MonitorState(topic_name, outcomes, monitor_handler, qos, msg_queue,
                     -1) {}

  /**
   * @brief Construct a new MonitorState with default QoS and message queue.
   *
   * @param topic_name The name of the topic to monitor.
   * @param outcomes A set of possible outcomes for this state.
   * @param monitor_handler A callback handler to process incoming messages.
   */
  MonitorState(std::string topic_name, std::set<std::string> outcomes,
               MonitorHandler monitor_handler)
      : MonitorState(topic_name, outcomes, monitor_handler, 10, 10, -1) {}

  /**
   * @brief Construct a new MonitorState with specific QoS, message queue, and
   * timeout.
   *
   * @param topic_name The name of the topic to monitor.
   * @param outcomes A set of possible outcomes for this state.
   * @param monitor_handler A callback handler to process incoming messages.
   * @param qos Quality of Service settings for the topic.
   * @param msg_queue The maximum number of messages to queue.
   * @param timeout The time in seconds to wait for messages before timing out.
   */
  MonitorState(std::string topic_name, std::set<std::string> outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos, int msg_queue,
               int timeout)
      : MonitorState(nullptr, topic_name, outcomes, monitor_handler, qos,
                     msg_queue, timeout) {}

  /**
   * @brief Construct a new MonitorState with ROS 2 node, specific QoS, message
   * queue, and timeout.
   *
   * @param node The ROS 2 node.
   * @param topic_name The name of the topic to monitor.
   * @param outcomes A set of possible outcomes for this state.
   * @param monitor_handler A callback handler to process incoming messages.
   * @param qos Quality of Service settings for the topic.
   * @param msg_queue The maximum number of messages to queue.
   * @param timeout The time in seconds to wait for messages before timing out.
   */
  MonitorState(const rclcpp::Node::SharedPtr &node, std::string topic_name,
               std::set<std::string> outcomes, MonitorHandler monitor_handler,
               rclcpp::QoS qos, int msg_queue, int timeout)
      : State({}), topic_name(topic_name), monitor_handler(monitor_handler),
        msg_queue(msg_queue), timeout(timeout), monitoring(false),
        time_to_wait(1000) {

    // set outcomes
    if (timeout > 0) {
      this->outcomes = {basic_outcomes::TIMEOUT};
    } else {
      this->outcomes = {};
    }

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.insert(outcome);
      }
    }

    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    // create subscriber
    this->sub = this->node_->template create_subscription<MsgT>(
        topic_name, qos, std::bind(&MonitorState::callback, this, _1));
  }

  /**
   * @brief Execute the monitoring operation and process the first received
   * message.
   *
   * @param blackboard A shared pointer to the blackboard for data storage.
   * @return A string outcome indicating the result of the monitoring operation.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    float elapsed_time = 0;
    this->msg_list.clear();
    this->monitoring = true;

    while (this->msg_list.empty()) {
      std::this_thread::sleep_for(
          std::chrono::microseconds(this->time_to_wait));

      if (this->timeout > 0) {

        if (elapsed_time / 1e6 >= this->timeout) {
          this->monitoring = false;
          RCLCPP_WARN(this->node_->get_logger(),
                      "Timeout reached, topic '%s' is not available",
                      this->topic_name.c_str());
          return basic_outcomes::TIMEOUT;
        }

        elapsed_time += this->time_to_wait;
      }
    }

    RCLCPP_INFO(this->node_->get_logger(), "Processing msg from topic '%s'",
                this->topic_name.c_str());
    std::string outcome =
        this->monitor_handler(blackboard, this->msg_list.at(0));
    this->msg_list.erase(this->msg_list.begin());

    this->monitoring = false;
    return outcome;
  }

private:
  rclcpp::Node::SharedPtr node_; /**< ROS 2 node pointer. */
  std::shared_ptr<rclcpp::Subscription<MsgT>>
      sub; /**< Subscription to the ROS 2 topic. */
  std::vector<std::shared_ptr<MsgT>>
      msg_list; /**< List to store queued messages. */

  std::string topic_name; /**< Name of the topic to monitor. */
  MonitorHandler
      monitor_handler; /**< Callback function to handle incoming messages. */
  int msg_queue;       /**< Maximum number of messages to queue. */
  int timeout;         /**< Timeout in seconds for message reception. */
  bool monitoring;     /**< Flag to control message monitoring state. */
  int time_to_wait;    /**< Time in microseconds to wait between checks. */

  /**
   * @brief Callback function for receiving messages from the subscribed topic.
   *
   * Adds the message to `msg_list` if monitoring is active, maintaining a
   * maximum queue size of `msg_queue`.
   *
   * @param msg The message received from the topic.
   */
  void callback(const typename MsgT::SharedPtr msg) {

    if (this->monitoring) {
      this->msg_list.push_back(msg);

      if ((int)this->msg_list.size() >= this->msg_queue) {
        this->msg_list.erase(this->msg_list.begin());
      }
    }
  }
};

} // namespace yasmin_ros

#endif
