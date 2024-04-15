// Copyright (C) 2024  Miguel Ángel González Santamarta

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

#ifndef YASMIN_ROS_YASMIN_NODE_HPP
#define YASMIN_ROS_YASMIN_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {

class YasminNode : public rclcpp::Node {

public:
  explicit YasminNode();

  YasminNode(YasminNode &other) = delete;
  void operator=(const YasminNode &) = delete;

  static YasminNode *get_instance() {

    std::lock_guard<std::mutex> lock(mutex);

    if (instance == nullptr) {
      instance = std::make_unique<YasminNode>();
    }

    return instance.get();
  }

private:
  inline static std::unique_ptr<YasminNode> instance;
  inline static std::mutex mutex;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::unique_ptr<std::thread> spin_thread;
};

} // namespace yasmin_ros

#endif
