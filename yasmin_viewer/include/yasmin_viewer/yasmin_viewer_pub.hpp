// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#ifndef YASMIN_VIEWER__YASMIN_VIEWER_PUB_HPP_
#define YASMIN_VIEWER__YASMIN_VIEWER_PUB_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/concurrence.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/types.hpp"
#include "yasmin_msgs/msg/state.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_msgs/msg/transition.hpp"

namespace yasmin_viewer {

/**
 * @class YasminViewerPub
 * @brief Publishes state machine data for visualization.
 */
class YasminViewerPub {

public:
  /**
   * @brief Constructs YasminViewerPub with a given ROS 2 node, state machine
   * name, and state machine instance.
   * @param node Shared pointer to the ROS 2 node.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   * @param fsm_name Name of the finite state machine.
   */
  YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                  yasmin::StateMachine::SharedPtr fsm,
                  const std::string &fsm_name);

  /**
   * @brief Constructs YasminViewerPub with a default ROS 2 node instance, state
   * machine name, and state machine instance.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   * @param fsm_name Name of the finite state machine.
   */
  YasminViewerPub(yasmin::StateMachine::SharedPtr fsm,
                  const std::string &fsm_name);

  /**
   * @brief Constructs YasminViewerPub with a given ROS 2 node and state
   * machine.
   * @param node Shared pointer to the ROS 2 node.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   */
  YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                  yasmin::StateMachine::SharedPtr fsm);

  /**
   * @brief Constructs YasminViewerPub with a default ROS 2 node instance and
   * state machine.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   */
  YasminViewerPub(yasmin::StateMachine::SharedPtr fsm);

  /**
   * @brief Parses transitions from a map of transitions and returns a list of
   * Transition messages.
   * @param transitions Map where keys are transition outcomes, and values are
   * the next states.
   * @return Vector of Transition messages.
   */
  std::vector<yasmin_msgs::msg::Transition>
  parse_transitions(const yasmin::Transitions &transitions) const;

  /**
   * @brief Parses concurrence transitions from outcome map to transition-like
   * information.
   * @param concurrence Shared pointer to the Concurrence state.
   * @return Map of state names to their transition vectors.
   */
  std::unordered_map<std::string, std::vector<yasmin_msgs::msg::Transition>>
  parse_concurrence_transitions(
      yasmin::Concurrence::SharedPtr concurrence) const;

  /**
   * @brief Parses a state and its transitions to add it to the list of state
   * messages.
   * @param name Name of the state to be parsed.
   * @param state Shared pointer to the State instance.
   * @param transitions Map of transitions associated with this state.
   * @param states_list Vector to which the parsed State message will be added.
   * @param parent ID of the parent state.
   */
  void parse_state(const std::string &name, yasmin::State::SharedPtr state,
                   const yasmin::Transitions &transitions,
                   std::vector<yasmin_msgs::msg::State> &states_list,
                   int parent);

  /**
   * @brief Publishes the data of the finite state machine to the associated ROS
   * topic.
   * @throws std::exception if state machine validation fails.
   */
  void publish_data();

private:
  /// Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;
  /// Publisher for StateMachine messages.
  rclcpp::Publisher<yasmin_msgs::msg::StateMachine>::SharedPtr publisher;
  /// Timer for periodic publishing.
  rclcpp::TimerBase::SharedPtr timer;

  /// Shared pointer to the state machine.
  yasmin::StateMachine::SharedPtr fsm;
  /// Name of the finite state machine.
  std::string fsm_name;
};

} // namespace yasmin_viewer

#endif // YASMIN_VIEWER__YASMIN_VIEWER_PUB_HPP_
