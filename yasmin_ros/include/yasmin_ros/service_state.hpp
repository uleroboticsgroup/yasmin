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

#ifndef YASMIN_ROS__SERVICE_STATE_HPP
#define YASMIN_ROS__SERVICE_STATE_HPP

#include <functional>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief A state class that interacts with a ROS 2 service.
 *
 * This class manages communication with a specified ROS 2 service,
 * allowing it to send requests and handle responses. It extends
 * the base `yasmin::State` class.
 *
 * @tparam ServiceT The type of the ROS 2 service this state interacts with.
 */
template <typename ServiceT> class ServiceState : public yasmin::State {
  /// Alias for the service request type.
  using Request = typename ServiceT::Request::SharedPtr;
  /// Alias for the service response type.
  using Response = typename ServiceT::Response::SharedPtr;

  /// Function type for creating a request.
  using CreateRequestHandler =
      std::function<Request(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  /// Function type for handling a response.
  using ResponseHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Response)>;

public:
  /**
   * @brief Construct a ServiceState with a request handler and outcomes.
   *
   * @param srv_name The name of the service to call.
   * @param create_request_handler Function to create a service request.
   * @param outcomes A set of possible outcomes for this state.
   * @param timeout Maximum time to wait for the service to become available, in
   * seconds. Default is -1 (wait indefinitely).
   * @param maximum_retry (Optional) Maximum retries of the service if it
   * returns timeout. Default is 3.
   */
  ServiceState(std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::set<std::string> outcomes, int timeout = -1.0,
               int maximum_retry = 3)
      : ServiceState(srv_name, create_request_handler, outcomes, nullptr,
                     nullptr, timeout, maximum_retry) {}

  /**
   * @brief Construct a ServiceState with a request handler and outcomes.
   *
   * @param srv_name The name of the service to call.
   * @param create_request_handler Function to create a service request.
   * @param outcomes A set of possible outcomes for this state.
   * @param callback_group (Optional) The callback group for the subscription.
   * @param timeout Maximum time to wait for the service to become available, in
   * seconds. Default is -1 (wait indefinitely).
   * @param maximum_retry (Optional) Maximum retries of the service if it
   * returns timeout. Default is 3.
   */
  ServiceState(std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::set<std::string> outcomes,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int timeout = -1.0, int maximum_retry = 3)
      : ServiceState(srv_name, create_request_handler, outcomes, nullptr,
                     callback_group, timeout, maximum_retry) {}

  /**
   * @brief Construct a ServiceState with a request handler and response
   * handler.
   *
   * @param srv_name The name of the service to call.
   * @param create_request_handler Function to create a service request.
   * @param outcomes A set of possible outcomes for this state.
   * @param response_handler (Optional) Function to handle the service response.
   * @param timeout Maximum time to wait for the service to become available, in
   * seconds. Default is -1 (wait indefinitely).
   * @param maximum_retry (Optional) Maximum retries of the service if it
   * returns timeout. Default is 3.
   */
  ServiceState(std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::set<std::string> outcomes, ResponseHandler response_handler,
               int timeout = -1.0, int maximum_retry = 3)
      : ServiceState(nullptr, srv_name, create_request_handler, outcomes,
                     response_handler, nullptr, timeout, maximum_retry) {}

  /**
   * @brief Construct a ServiceState with a ROS 2 node and handlers.
   *
   * @param node A shared pointer to the ROS 2 node.
   * @param srv_name The name of the service to call.
   * @param create_request_handler Function to create a service request.
   * @param outcomes A set of possible outcomes for this state.
   * @param response_handler (Optional) Function to handle the service response.
   * @param callback_group (Optional) The callback group for the subscription.
   * @param timeout Maximum time to wait for the service to become available, in
   * seconds. Default is -1 (wait indefinitely).
   * @param maximum_retry (Optional) Maximum retries of the service if it
   * returns timeout. Default is 3.
   *
   * @throws std::invalid_argument if the create_request_handler is nullptr.
   */
  ServiceState(const rclcpp::Node::SharedPtr &node, std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::set<std::string> outcomes, ResponseHandler response_handler,
               rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
               int timeout = -1.0, int maximum_retry = 3)
      : State({}), srv_name(srv_name), timeout(timeout),
        maximum_retry(maximum_retry) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT};

    if (this->timeout >= 0) {
      this->outcomes.insert(basic_outcomes::TIMEOUT);
    }

    if (!outcomes.empty()) {
      this->outcomes.insert(outcomes.begin(), outcomes.end());
    }

    // Assign the appropriate ROS 2 node
    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    // Create a service client (compatible with different rclcpp versions)
#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(28, 1, 9)
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default));
#else
    auto qos = rmw_qos_profile_services_default;
#endif
#else
    auto qos = rmw_qos_profile_services_default;
#endif

    this->service_client = this->node_->template create_client<ServiceT>(
        srv_name, qos, callback_group);

    // Set the request and response handlers
    this->create_request_handler = create_request_handler;
    this->response_handler = response_handler;

    // Validate request handler
    if (this->create_request_handler == nullptr) {
      throw std::invalid_argument("create_request_handler is needed");
    }
  }

  /**
   * @brief Execute the service call and handle the response.
   *
   * This function creates a request based on the blackboard data, waits for the
   * service to become available, sends the request, and processes the response.
   *
   * @param blackboard A shared pointer to the blackboard containing data for
   * request creation.
   * @return std::string The outcome of the service call, which can be SUCCEED,
   * ABORT, or TIMEOUT.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    this->retry_count++;
    Request request = this->create_request(blackboard);

    // Wait for the service to become available
    YASMIN_LOG_INFO("Waiting for service '%s'", this->srv_name.c_str());
    bool srv_available = this->service_client->wait_for_service(
        std::chrono::duration<int64_t, std::ratio<1>>(this->timeout));

    if (!srv_available) {
      YASMIN_LOG_WARN("Timeout reached, service '%s' is not available",
                      this->srv_name.c_str());
      // Auto retry process
      if (this->retry_count < this->maximum_retry) {
        return this->execute(blackboard);
      } else {
        return basic_outcomes::TIMEOUT;
      }
    }

    // Send the service request
    YASMIN_LOG_INFO("Sending request to service '%s'", this->srv_name.c_str());
    auto future = this->service_client->async_send_request(request);

    // Add waiting timeout
    if (future.wait_for(std::chrono::seconds(this->timeout)) !=
        std::future_status::ready) {
      // Auto retry process
      if (this->retry_count < this->maximum_retry) {
        return this->execute(blackboard);
      } else {
        this->retry_count = 0;
        return basic_outcomes::TIMEOUT;
      }
    } else {
      this->retry_count = 0;
    }

    if (this->is_canceled()) {
      return basic_outcomes::CANCEL;
    }

    // Wait for the response
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

protected:
  /// Shared pointer to the ROS 2 node
  rclcpp::Node::SharedPtr node_;

private:
  /// Shared pointer to the service client.
  std::shared_ptr<rclcpp::Client<ServiceT>> service_client;
  /// Function to create service requests.
  CreateRequestHandler create_request_handler;
  /// Function to handle service responses.
  ResponseHandler response_handler;
  /// Name of the service.
  std::string srv_name;
  /// Maximum wait time for service availability.
  int timeout;
  int maximum_retry;   /**< Maximum number of retries. */
  int retry_count = 0; /**< Number of retries. */

  /**
   * @brief Create a service request based on the blackboard.
   *
   * @param blackboard A shared pointer to the blackboard containing data for
   * request creation.
   * @return Request The created service request.
   */
  Request
  create_request(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    return this->create_request_handler(blackboard);
  }
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__SERVICE_STATE_HPP
