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

#include <chrono>
#include <cinttypes>
#include <memory>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @class ServerNode
 * @brief A ROS 2 service server node for adding two integers.
 *
 * This node provides a service named "add_two_ints" that accepts two integers
 * as input and returns their sum. It also supports an optional one-shot mode
 * that shuts down the server after handling the first request.
 */
class ServerNode final : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the ServerNode class.
   *
   * Initializes the service server and an optional one-shot timer.
   * @param options Node options for initialization.
   */
  explicit ServerNode() : Node("add_two_ints_server") {

    // Callback to handle "add_two_ints" service requests.
    auto handle_add_two_ints =
        [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const example_interfaces::srv::AddTwoInts::Request::SharedPtr
                request,
            example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        -> void {
      (void)request_header;
      RCLCPP_INFO(this->get_logger(),
                  "Incoming request\na: %" PRId64 " b: %" PRId64, request->a,
                  request->b);
      response->sum = request->a + request->b;
    };

    // Create a service that will use the callback function to handle requests.
    this->srv_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", handle_add_two_ints);

    RCLCPP_INFO(this->get_logger(), "Add Two Ints Service started");
  }

private:
  /**
   * @brief Shared pointer to the "add_two_ints" service.
   */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
