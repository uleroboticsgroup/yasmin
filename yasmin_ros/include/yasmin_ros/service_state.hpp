
#ifndef YASMIN_ROS_SERVICE_STATE_HPP
#define YASMIN_ROS_SERVICE_STATE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "simple_node/actions/action_client.hpp"
#include "simple_node/node.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {

template <typename ServiceT> class ServiceState : public yasmin::State {

  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using CreateRequestHandler =
      std::function<Request(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  using ResponseHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Response)>;

public:
  ServiceState(simple_node::Node *node, std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::vector<std::string> outcomes)
      : ServiceState(node, srv_name, create_request_handler, outcomes,
                     nullptr) {}

  ServiceState(simple_node::Node *node, std::string srv_name,
               CreateRequestHandler create_request_handler,
               std::vector<std::string> outcomes,
               ResponseHandler response_handler)
      : State({}) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT};

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.push_back(outcome);
      }
    }

    this->service_client = node->create_client<ServiceT>(srv_name);

    this->create_request_handler = create_request_handler;
    this->response_handler = response_handler;

    if (this->create_request_handler == nullptr) {
      throw std::invalid_argument("create_request_handler is needed");
    }
  }

  std::string execute(yasmin::blackboard::Blackboard blackboard) override {

    Request request = this->create_request(blackboard);

    this->service_client->wait_for_service();
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
  std::shared_ptr<rclcpp::Client<ServiceT>> service_client;
  CreateRequestHandler create_request_handler;
  ResponseHandler response_handler;

  Request create_request(yasmin::blackboard::Blackboard blackboard) {
    return this->create_request_handler(blackboard);
  }
};

} // namespace yasmin_ros

#endif
