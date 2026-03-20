// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_
#define YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yasmin_msgs/msg/state.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_msgs/msg/transition.hpp"

namespace yasmin_viewer {

class YasminViewerNode : public rclcpp::Node {
public:
  YasminViewerNode();
  ~YasminViewerNode() override;

  YasminViewerNode(const YasminViewerNode &) = delete;
  YasminViewerNode &operator=(const YasminViewerNode &) = delete;

  std::string get_fsms_json() const;
  std::string get_web_root() const;

private:
  struct CachedFsm {
    std::string json;
    std::chrono::steady_clock::time_point timestamp;
  };

  using StateMachineMsg = yasmin_msgs::msg::StateMachine;
  using StateMsg = yasmin_msgs::msg::State;
  using TransitionMsg = yasmin_msgs::msg::Transition;

  void fsm_viewer_cb(const StateMachineMsg::SharedPtr msg);
  void start_server();
  void stop_server();
  void run_server();
  void
  prune_expired_locked(const std::chrono::steady_clock::time_point &now) const;

  static std::string escape_json(const std::string &value);
  static std::string
  transitions_to_json(const std::vector<TransitionMsg> &transitions);
  static std::string state_to_json(const StateMsg &state);
  static std::string state_machine_to_json(const StateMachineMsg &msg);

  mutable std::mutex fsms_mutex_;
  mutable std::unordered_map<std::string, CachedFsm> fsms_;

  rclcpp::Subscription<StateMachineMsg>::SharedPtr fsm_sub_;

  std::string host_;
  std::string web_root_;
  int64_t port_;
  double max_age_seconds_;

  std::atomic_bool server_running_;
  std::thread server_thread_;
};

} // namespace yasmin_viewer

#endif // YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_
