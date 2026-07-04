// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__MONITOR_STATE_HPP_
#define YASMIN_ROS__MONITOR_STATE_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

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

  /// @brief Function type for handling messages from topic.
  using MonitorHandler = std::function<std::string(
      yasmin::Blackboard::SharedPtr, std::shared_ptr<MsgT>)>;

public:
  /**
   * @brief Shared pointer type for MonitorState.
   */
  YASMIN_PTR_ALIASES(MonitorState)

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
   * @param maximum_retry Maximum retries of the monitor if it returns timeout.
   * Default is 3.
   */
  MonitorState(const std::string &topic_name, const yasmin::Outcomes &outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos = 10,
               int msg_queue = 10, int timeout = -1, int maximum_retry = 3)
      : MonitorState(nullptr, topic_name, outcomes, monitor_handler, qos,
                     nullptr, msg_queue, timeout, maximum_retry) {}

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
   * @param maximum_retry Maximum retries of the monitor if it returns timeout.
   * Default is 3.
   *
   */
  MonitorState(const std::string &topic_name, const yasmin::Outcomes &outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos = 10,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int msg_queue = 10, int timeout = -1, int maximum_retry = 3)
      : MonitorState(nullptr, topic_name, outcomes, monitor_handler, qos,
                     callback_group, msg_queue, timeout, maximum_retry) {}

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
   * @param maximum_retry Maximum retries of the monitor if it returns timeout.
   * Default is 3.
   */
  MonitorState(const rclcpp::Node::SharedPtr &node,
               const std::string &topic_name, const yasmin::Outcomes &outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos = 10,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int msg_queue = 10, int timeout = -1, int maximum_retry = 3)
      : State(outcomes), topic_name(topic_name),
        monitor_handler(monitor_handler), qos(qos), msg_queue(msg_queue),
        timeout(timeout), maximum_retry(maximum_retry) {

    // Set outcomes
    if (timeout > 0) {
      this->outcomes.insert(basic_outcomes::TIMEOUT);
      this->set_outcome_description(
          basic_outcomes::TIMEOUT,
          "No message received from topic in the specified timeout");
    }
    this->outcomes.insert(basic_outcomes::CANCEL);
    this->set_outcome_description(basic_outcomes::CANCEL,
                                  "The monitor was canceled");

    if (outcomes.size() > 0) {
      for (const std::string &outcome : outcomes) {
        this->outcomes.insert(outcome);
      }
    }

    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    // Create subscription
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;
    this->sub = this->node_->create_subscription<MsgT>(
        this->topic_name, this->qos,
        std::bind(&MonitorState::callback, this, std::placeholders::_1),
        options);
  }

  /**
   * @brief Execute the monitoring operation and process the first received
   * message.
   *
   * @param blackboard A shared pointer to the blackboard for data storage.
   * @return A string outcome indicating the result of the monitoring operation.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    int retry_count = 0;
    std::unique_lock<std::mutex> lock(this->msg_mutex);
    std::cv_status wait_status = std::cv_status::no_timeout;

    while (this->msg_list.empty()) {

      if (this->timeout > 0) {
        const auto timeout_dur = std::chrono::seconds(this->timeout);
        wait_status = this->msg_cond.wait_for(lock, timeout_dur);
      } else {
        this->msg_cond.wait(lock, [this]() {
          return !this->msg_list.empty() || this->is_canceled();
        });
      }

      if (this->is_canceled()) {
        return basic_outcomes::CANCEL;
      }

      while (this->timeout > 0 && wait_status == std::cv_status::timeout) {
        YASMIN_LOG_WARN("Timeout reached, topic '%s' is not available",
                        this->topic_name.c_str());

        if (retry_count < this->maximum_retry) {
          retry_count++;
          YASMIN_LOG_WARN("Retrying to wait for topic '%s' (%d/%d)",
                          this->topic_name.c_str(), retry_count,
                          this->maximum_retry);
          const auto timeout_dur = std::chrono::seconds(this->timeout);
          wait_status = this->msg_cond.wait_for(lock, timeout_dur);
        } else {
          return basic_outcomes::TIMEOUT;
        }

        if (this->is_canceled()) {
          return basic_outcomes::CANCEL;
        }
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
  void cancel_state() override {
    this->msg_cond.notify_one();
    yasmin::State::cancel_state();
  }

protected:
  /// @brief Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;

private:
  /// @brief Subscription to the ROS 2 topic.
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub;

  /// @brief Name of the topic to monitor.
  std::string topic_name;
  /// @brief Quality of Service settings for the topic.
  rclcpp::QoS qos;

  /// @brief List to store queued messages.
  std::vector<std::shared_ptr<MsgT>> msg_list;
  /// @brief Callback function to handle incoming messages.
  MonitorHandler monitor_handler;
  /// @brief Maximum number of messages to queue.
  int msg_queue;
  /// @brief Timeout in seconds for message reception.
  int timeout;
  /// @brief Maximum number of retries.
  int maximum_retry;

  /// @brief Condition variable for action completion.
  std::condition_variable msg_cond;
  /// @brief Mutex for protecting action completion.
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
    std::lock_guard<std::mutex> lock(this->msg_mutex);

    this->msg_list.push_back(msg);

    if ((int)this->msg_list.size() > this->msg_queue) {
      this->msg_list.erase(this->msg_list.begin());
    }

    this->msg_cond.notify_one();
  }
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__MONITOR_STATE_HPP_
