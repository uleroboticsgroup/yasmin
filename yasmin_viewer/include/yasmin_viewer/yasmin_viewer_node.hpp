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

#ifndef YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_
#define YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
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

/** @brief Forward declaration for the HTTP server acceptor. */
struct AcceptorHolder;

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

  /** @brief Deleted copy constructor (non-copyable). */
  YasminViewerNode(const YasminViewerNode &) = delete;
  /** @brief Deleted copy assignment (non-copyable). */
  YasminViewerNode &operator=(const YasminViewerNode &) = delete;

  /**
   * @brief Returns the cached finite state machines as a JSON string.
   * @return JSON representation of all cached finite state machines.
   */
  std::string get_fsms_json();

  /**
   * @brief Returns the configured web root directory.
   * @return Path to the web root directory.
   */
  std::string get_web_root() const;

  /** @brief Callback invoked when an HTTP connection is closed. */
  void on_connection_closed() {
    this->active_connections_.fetch_sub(1, std::memory_order_relaxed);
  }

private:
  /**
   * @struct CachedFsm
   * @brief Stores a serialized finite state machine and its last update time.
   */
  struct CachedFsm {
    /// @brief Cached JSON representation of the FSM
    std::string json;
    /// @brief Timestamp when this cache entry was created
    std::chrono::steady_clock::time_point timestamp;
  };

  /// @brief Alias for the StateMachine ROS2 message type
  using StateMachineMsg = yasmin_msgs::msg::StateMachine;
  /// @brief Alias for the State ROS2 message type
  using StateMsg = yasmin_msgs::msg::State;
  /// @brief Alias for the Transition ROS2 message type
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
  void prune_expired_locked(const std::chrono::steady_clock::time_point &now);

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
  std::unordered_map<std::string, CachedFsm> fsms_;

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
  /// Maximum number of concurrent HTTP connections.
  static constexpr uint32_t kMaxConnections = 64;
  /// Number of currently active HTTP connections.
  std::atomic<uint32_t> active_connections_{0};

  /// Asio acceptor holder (created in start_server, used by
  /// run_server/stop_server).
  std::unique_ptr<AcceptorHolder> acceptor_holder_;
};

} // namespace yasmin_viewer

#endif // YASMIN_VIEWER__YASMIN_VIEWER_NODE_HPP_
