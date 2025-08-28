// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__MONITOR_STATE_HPP
#define YASMIN_ROS__MONITOR_STATE_HPP

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"
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
               MonitorHandler monitor_handler, rclcpp::QoS qos = 10,
               int msg_queue = 10, int timeout = -1)
      : MonitorState(nullptr, topic_name, outcomes, monitor_handler, qos,
                     nullptr, msg_queue, timeout) {}

  /**
   * @brief Construct a new MonitorState with specific QoS, message queue, and
   * timeout.
   *
   * @param topic_name The name of the topic to monitor.
   * @param outcomes A set of possible outcomes for this state.
   * @param monitor_handler A callback handler to process incoming messages.
   * @param qos Quality of Service settings for the topic.
   * @param callback_group The callback group for the subscription.
   * @param msg_queue The maximum number of messages to queue.
   * @param timeout The time in seconds to wait for messages before timing out.
   */
  MonitorState(std::string topic_name, std::set<std::string> outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos = 10,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int msg_queue = 10, int timeout = -1)
      : MonitorState(nullptr, topic_name, outcomes, monitor_handler, qos,
                     callback_group, msg_queue, timeout) {}

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
               rclcpp::QoS qos = 10,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int msg_queue = 10, int timeout = -1)
      : State({}), topic_name(topic_name), monitor_handler(monitor_handler),
        qos(qos), msg_queue(msg_queue), timeout(timeout) {

    // set outcomes
    if (timeout > 0) {
      this->outcomes = {basic_outcomes::TIMEOUT};
    } else {
      this->outcomes = {};
    }
    this->outcomes.insert(basic_outcomes::CANCEL);

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

    // Options for the subscription
    this->options.callback_group = callback_group;

    // Crate subscription
    this->sub = this->node_->template create_subscription<MsgT>(
        this->topic_name, this->qos,
        std::bind(&MonitorState::callback, this, _1), this->options);
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

    while (this->msg_list.empty()) {
      std::unique_lock<std::mutex> lock(this->msg_mutex);
      auto status =
          this->msg_cond.wait_for(lock, std::chrono::seconds(this->timeout));

      if (this->is_canceled()) {
        return basic_outcomes::CANCEL;
      }

      if (this->timeout > 0 && status == std::cv_status::timeout) {
        YASMIN_LOG_WARN("Timeout reached, topic '%s' is not available",
                        this->topic_name.c_str());
        return basic_outcomes::TIMEOUT;
      }
    }

    YASMIN_LOG_INFO("Processing msg from topic '%s'", this->topic_name.c_str());
    std::string outcome =
        this->monitor_handler(blackboard, this->msg_list.at(0));
    this->msg_list.erase(this->msg_list.begin());

    return outcome;
  }

  /**
   * @brief Cancel the current monitor state.
   *
   * This function cancels the ongoing monitor.
   */
  void cancel_state() {
    yasmin::State::cancel_state();
    this->msg_cond.notify_one();
  }

private:
  rclcpp::Node::SharedPtr node_; /**< ROS 2 node pointer. */
  std::shared_ptr<rclcpp::Subscription<MsgT>>
      sub; /**< Subscription to the ROS 2 topic. */

  std::string topic_name; /**< Name of the topic to monitor. */
  rclcpp::QoS qos;        /**< Quality of Service settings for the topic. */
  rclcpp::SubscriptionOptions options; /**< Options for the subscription. */

  std::vector<std::shared_ptr<MsgT>>
      msg_list; /**< List to store queued messages. */
  MonitorHandler
      monitor_handler; /**< Callback function to handle incoming messages. */
  int msg_queue;       /**< Maximum number of messages to queue. */
  int timeout;         /**< Timeout in seconds for message reception. */

  /// Condition variable for action completion.
  std::condition_variable msg_cond;
  /// Mutex for protecting action completion.
  std::mutex msg_mutex;

  /**
   * @brief Callback function for receiving messages from the subscribed topic.
   *
   * Adds the message to `msg_list` maintaining a maximum queue size of
   * `msg_queue`.
   *
   * @param msg The message received from the topic.
   */
  void callback(const typename MsgT::SharedPtr msg) {

    this->msg_list.push_back(msg);

    if ((int)this->msg_list.size() > this->msg_queue) {
      this->msg_list.erase(this->msg_list.begin());
    }

    this->msg_cond.notify_one();
  }
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__MONITOR_STATE_HPP
