// Copyright (C) 2026 Maik Knof
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

#include <rclcpp/rclcpp.hpp>

#include "yasmin_viewer/yasmin_viewer_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<yasmin_viewer::YasminViewerNode> node;

  try {
    node = std::make_shared<yasmin_viewer::YasminViewerNode>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("yasmin_viewer"),
                 "Failed to start YASMIN viewer: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
