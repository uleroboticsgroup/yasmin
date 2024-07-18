// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_ROS_SERVICE_STATE_HPP
#define YASMIN_ROS_SERVICE_STATE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

template <typename ServiceT> class ServiceState : public yasmin::State {

  using Request = typename ServiceT::Request::SharedPtr;
  using Response = typename ServiceT::Response::SharedPtr;
  using CreateRequestHandler =
      std::function<Request(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  using ResponseHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Response)>;

public:
  ServiceState(std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::vector<std::string> outcomes, int timeout = -1.0)
      : ServiceState(srv_name, create_request_handler, outcomes, nullptr,
                     timeout) {}

  ServiceState(std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::vector<std::string> outcomes,
               ResponseHandler response_handler, int timeout = -1.0)
      : ServiceState(nullptr, srv_name, create_request_handler, outcomes,
                     response_handler, timeout) {}

  ServiceState(rclcpp::Node *node, std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::vector<std::string> outcomes,
               ResponseHandler response_handler, int timeout = -1.0)
      : State({}), srv_name(srv_name), timeout(timeout) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT};

    if (this->timeout >= 0) {
      this->outcomes.push_back(basic_outcomes::TIMEOUT);
    }

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.push_back(outcome);
      }
    }

    if (node == nullptr) {
      this->node = YasminNode::get_instance();
    } else {
      this->node = node;
    }

    this->service_client =
        this->node->template create_client<ServiceT>(srv_name);

    this->create_request_handler = create_request_handler;
    this->response_handler = response_handler;

    if (this->create_request_handler == nullptr) {
      throw std::invalid_argument("create_request_handler is needed");
    }
  }

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    Request request = this->create_request(blackboard);

    // wait for server
    RCLCPP_INFO(this->node->get_logger(), "Waiting for service '%s'",
                this->srv_name.c_str());
    bool srv_available = this->service_client->wait_for_service(
        std::chrono::duration<int64_t, std::ratio<1>>(this->timeout));

    if (!srv_available) {
      RCLCPP_WARN(this->node->get_logger(),
                  "Timeout reached, service '%s' is not available",
                  this->srv_name.c_str());
      return basic_outcomes::TIMEOUT;
    }

    RCLCPP_INFO(this->node->get_logger(), "Sending request to service '%s'",
                this->srv_name.c_str());
    auto future = this->service_client->async_send_request(request);

    future.wait();
    Response response = future.get();

    if (response) {

      if (response_handler != nullptr) {
        std::string outcome = this->response_handler(blackboard, response);
        return outcome;
      }

      return basic_outcomes::SUCCEED;

    } else {
      return basic_outcomes::ABORT;
    }
  }

private:
  rclcpp::Node *node;
  std::shared_ptr<rclcpp::Client<ServiceT>> service_client;
  CreateRequestHandler create_request_handler;
  ResponseHandler response_handler;
  std::string srv_name;
  int timeout;

  Request
  create_request(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    return this->create_request_handler(blackboard);
  }
};

} // namespace yasmin_ros

#endif
