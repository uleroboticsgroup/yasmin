
#ifndef YASMIN_ROS_MONITOR_STATE_HPP
#define YASMIN_ROS_MONITOR_STATE_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "simple_node/node.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

using std::placeholders::_1;

namespace yasmin_ros {

template <typename MsgT> class MonitorState : public yasmin::State {

  using MonitorHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>)>;

public:
  MonitorState(simple_node::Node *node, std::string topic_name,
               std::vector<std::string> outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos, int msg_queue)
      : MonitorState(node, topic_name, outcomes, monitor_handler, qos,
                     msg_queue, -1) {}

  MonitorState(simple_node::Node *node, std::string topic_name,
               std::vector<std::string> outcomes,
               MonitorHandler monitor_handler)
      : MonitorState(node, topic_name, outcomes, monitor_handler, 10, 10, -1) {}

  MonitorState(simple_node::Node *node, std::string topic_name,
               std::vector<std::string> outcomes,
               MonitorHandler monitor_handler, rclcpp::QoS qos, int msg_queue,
               int timeout)
      : State({}) {

    // set outcomes
    if (timeout > 0) {
      this->outcomes = {basic_outcomes::CANCEL};
    } else {
      this->outcomes = {};
    }

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.push_back(outcome);
      }
    }

    // other attributes
    this->monitor_handler = monitor_handler;
    this->topic_name = topic_name;
    this->msg_queue = msg_queue;
    this->timeout = timeout;
    this->time_to_wait = 1000;
    this->monitoring = false;

    // create subscriber
    this->sub = node->create_subscription<MsgT>(
        topic_name, qos, std::bind(&MonitorState::callback, this, _1));
  }

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    float elapsed_time = 0;
    this->msg_list.clear();
    this->monitoring = true;

    while (this->msg_list.size() == 0) {
      std::this_thread::sleep_for(
          std::chrono::microseconds(this->time_to_wait));

      if (this->timeout > 0) {

        if (elapsed_time >= this->timeout) {
          this->monitoring = false;
          return basic_outcomes::CANCEL;
        }
      }

      elapsed_time += this->time_to_wait;
    }

    blackboard->set<MsgT>(this->topic_name + "_msg", this->msg_list.at(0));
    this->msg_list.erase(this->msg_list.begin());

    std::string outcome = this->monitor_handler(blackboard);
    this->monitoring = false;
    return outcome;
  }

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub;

  MonitorHandler monitor_handler;
  std::string topic_name;
  std::vector<MsgT> msg_list;
  int msg_queue;
  int timeout;
  int time_to_wait;
  bool monitoring;

  void callback(const typename MsgT::SharedPtr msg) {

    if (this->monitoring) {
      this->msg_list.push_back(*msg.get());

      if (this->msg_list.size() >= this->msg_queue) {
        this->msg_list.erase(this->msg_list.begin());
      }
    }
  }
};

} // namespace yasmin_ros

#endif
