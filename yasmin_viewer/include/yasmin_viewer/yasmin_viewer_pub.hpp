// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_VIEWER_PUB_HPP
#define YASMIN_VIEWER_PUB_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
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
   * @param fsm_name Name of the finite state machine.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   */
  YasminViewerPub(const rclcpp::Node::SharedPtr &node, std::string fsm_name,
                  std::shared_ptr<yasmin::StateMachine> fsm);

  /**
   * @brief Constructs YasminViewerPub with a default ROS 2 node instance, state
   * machine name, and state machine instance.
   * @param fsm_name Name of the finite state machine.
   * @param fsm Shared pointer to the StateMachine instance to be published.
   */
  YasminViewerPub(std::string fsm_name,
                  std::shared_ptr<yasmin::StateMachine> fsm);

  /**
   * @brief Parses transitions from a map of transitions and returns a list of
   * Transition messages.
   * @param transitions Map where keys are transition outcomes, and values are
   * the next states.
   * @return Vector of Transition messages.
   */
  std::vector<yasmin_msgs::msg::Transition>
  parse_transitions(std::map<std::string, std::string> transitions);

  /**
   * @brief Parses a state and its transitions to add it to the list of state
   * messages.
   * @param name Name of the state to be parsed.
   * @param state Shared pointer to the State instance.
   * @param transitions Map of transitions associated with this state.
   * @param states_list Vector to which the parsed State message will be added.
   * @param parent ID of the parent state.
   */
  void parse_state(std::string name, std::shared_ptr<yasmin::State> state,
                   std::map<std::string, std::string> transitions,
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

  /// Name of the finite state machine.
  std::string fsm_name;
  /// Shared pointer to the state machine.
  std::shared_ptr<yasmin::StateMachine> fsm;
};

} // namespace yasmin_viewer

#endif
