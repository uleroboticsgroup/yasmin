// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

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
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
                request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>
                response) -> void {
      (void)request_header;
      RCLCPP_INFO(this->get_logger(),
                  "Incoming request\na: %" PRId64 " b: %" PRId64, request->a,
                  request->b);
      response->sum = request->a + request->b;
    };

    // Create a service that will use the callback function to handle requests.
    this->srv_ = create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", handle_add_two_ints);

    RCLCPP_INFO(this->get_logger(), "Add Two Ints Service started");
  }

private:
  /**
   * @brief Shared pointer to the "add_two_ints" service.
   */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

/**
 * @brief Main entry point for the ServerNode application.
 *
 * Initializes the ROS 2 node, spins it to handle incoming service requests,
 * and shuts down gracefully when done.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
