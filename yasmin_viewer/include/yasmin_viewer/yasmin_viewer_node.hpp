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

#include <rclcpp/rclcpp.hpp>

#include "yasmin_msgs/msg/state.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_msgs/msg/transition.hpp"

namespace yasmin_viewer {

/**
 * @class YasminViewerNode
 * @brief ROS 2 node that receives state machine descriptions and exposes them
 * for the web viewer.
 */
class YasminViewerNode : public rclcpp::Node {
public:
  /**
   * @brief Constructs the YasminViewerNode and initializes subscriptions,
   * parameters, and the web server.
   */
  YasminViewerNode();

  /**
   * @brief Destroys the YasminViewerNode and stops the web server.
   */
  ~YasminViewerNode() override;

  YasminViewerNode(const YasminViewerNode &) = delete;
  YasminViewerNode &operator=(const YasminViewerNode &) = delete;

  /**
   * @brief Returns the cached finite state machines as a JSON string.
   * @return JSON representation of all cached finite state machines.
   */
  std::string get_fsms_json() const;

  /**
   * @brief Returns the configured web root directory.
   * @return Path to the web root directory.
   */
  std::string get_web_root() const;

private:
  /**
   * @struct CachedFsm
   * @brief Stores a serialized finite state machine and its last update time.
   */
  struct CachedFsm {
    std::string json;
    std::chrono::steady_clock::time_point timestamp;
  };

  using StateMachineMsg = yasmin_msgs::msg::StateMachine;
  using StateMsg = yasmin_msgs::msg::State;
  using TransitionMsg = yasmin_msgs::msg::Transition;

  /**
   * @brief Callback for incoming state machine messages.
   * @param msg Received state machine message.
   */
  void fsm_viewer_cb(const StateMachineMsg::SharedPtr msg);

  /**
   * @brief Starts the embedded web server thread.
   */
  void start_server();

  /**
   * @brief Stops the embedded web server thread.
   */
  void stop_server();

  /**
   * @brief Runs the embedded web server loop.
   */
  void run_server();

  /**
   * @brief Removes expired finite state machines from the cache.
   * @param now Current time used for age comparison.
   */
  void
  prune_expired_locked(const std::chrono::steady_clock::time_point &now) const;

  /**
   * @brief Escapes a string for safe JSON serialization.
   * @param value Input string.
   * @return Escaped JSON string.
   */
  static std::string escape_json(const std::string &value);

  /**
   * @brief Serializes a list of transitions into JSON.
   * @param transitions List of transitions.
   * @return JSON representation of the transitions.
   */
  static std::string
  transitions_to_json(const std::vector<TransitionMsg> &transitions);

  /**
   * @brief Serializes a state into JSON.
   * @param state State message to serialize.
   * @return JSON representation of the state.
   */
  static std::string state_to_json(const StateMsg &state);

  /**
   * @brief Serializes a full state machine message into JSON.
   * @param msg State machine message to serialize.
   * @return JSON representation of the state machine.
   */
  static std::string state_machine_to_json(const StateMachineMsg &msg);

  /// Mutex protecting access to the finite state machine cache.
  mutable std::mutex fsms_mutex_;
  /// Cache of serialized finite state machines indexed by name.
  mutable std::unordered_map<std::string, CachedFsm> fsms_;

  /// Subscription for incoming state machine descriptions.
  rclcpp::Subscription<StateMachineMsg>::SharedPtr fsm_sub_;

  /// Host address used by the embedded web server.
  std::string host_;
  /// Root directory containing the web viewer files.
  std::string web_root_;
  /// Port used by the embedded web server.
  int64_t port_;
  /// Maximum cache age in seconds before a finite state machine is removed.
  double max_age_seconds_;

  /// Indicates whether the embedded web server is running.
  std::atomic_bool server_running_;
  /// Background thread executing the embedded web server.
  std::thread server_thread_;
};

} // namespace yasmin_viewer

#endif // YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_
