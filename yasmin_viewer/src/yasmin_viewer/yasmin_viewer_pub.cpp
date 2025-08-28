// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include "yasmin/logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin_viewer;
using namespace std::chrono_literals;
using namespace yasmin;

/**
 * @brief Constructs YasminViewerPub by delegating to another constructor with a
 * nullptr node.
 */
YasminViewerPub::YasminViewerPub(std::string fsm_name,
                                 std::shared_ptr<yasmin::StateMachine> fsm)
    : YasminViewerPub(nullptr, fsm_name, fsm) {}

/**
 * @brief Constructs YasminViewerPub with a specific ROS node.
 * If the node is nullptr, a singleton instance of YasminNode is used.
 */
YasminViewerPub::YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                                 std::string fsm_name,
                                 std::shared_ptr<yasmin::StateMachine> fsm)
    : fsm_name(fsm_name), fsm(fsm) {

  if (node == nullptr) {
    this->node_ = yasmin_ros::YasminNode::get_instance();
  } else {
    this->node_ = node;
  }

  this->publisher =
      this->node_->create_publisher<yasmin_msgs::msg::StateMachine>(
          "/fsm_viewer", 10);

  this->timer = this->node_->create_wall_timer(
      250ms, std::bind(&YasminViewerPub::publish_data, this));
}

/**
 * @brief Parses a map of transition pairs (outcome, state) to a vector of
 * Transition messages.
 * @param transitions Map containing transition outcomes as keys and next states
 * as values.
 * @return Vector of yasmin_msgs::msg::Transition.
 */
std::vector<yasmin_msgs::msg::Transition> YasminViewerPub::parse_transitions(
    std::map<std::string, std::string> transitions) {
  std::vector<yasmin_msgs::msg::Transition> transitions_list;

  for (auto const &transition : transitions) {
    auto transition_msg = yasmin_msgs::msg::Transition();
    transition_msg.outcome = transition.first;
    transition_msg.state = transition.second;
    transitions_list.push_back(transition_msg);
  }
  return transitions_list;
}

/**
 * @brief Parses a state and its transitions, adding it to the states list and
 * handling nested FSMs if applicable.
 * @param state_name Name of the state.
 * @param state Shared pointer to the State instance.
 * @param transitions Map of transitions related to the state.
 * @param states_list Reference to a vector of yasmin_msgs::msg::State to append
 * parsed states.
 * @param parent ID of the parent state; -1 for top-level states.
 */
void YasminViewerPub::parse_state(
    std::string state_name, std::shared_ptr<yasmin::State> state,
    std::map<std::string, std::string> transitions,
    std::vector<yasmin_msgs::msg::State> &states_list, int parent) {

  auto state_msg = yasmin_msgs::msg::State();

  state_msg.id = states_list.size();
  state_msg.parent = parent;

  // State information
  state_msg.name = state_name;
  state_msg.transitions = this->parse_transitions(transitions);
  auto outcomes = state->get_outcomes();
  state_msg.outcomes =
      std::vector<std::string>(outcomes.begin(), outcomes.end());

  // Check if state is a nested FSM
  auto fsm = std::dynamic_pointer_cast<yasmin::StateMachine>(state);
  state_msg.is_fsm = (fsm != nullptr);

  // Add the state to the list
  states_list.push_back(state_msg);

  // Handle nested FSM states
  if (state_msg.is_fsm) {
    auto states = fsm->get_states();
    auto aux_transitions = fsm->get_transitions();

    for (const auto &nested_state : states) {
      this->parse_state(nested_state.first, nested_state.second,
                        aux_transitions[nested_state.first], states_list,
                        state_msg.id);
    }

    // Identify the current state in the FSM
    std::string current_state = fsm->get_current_state();
    for (auto &child_state : states_list) {
      if (child_state.name == current_state &&
          child_state.parent == state_msg.id) {
        states_list[state_msg.id].current_state = child_state.id;
        break;
      }
    }
  }
}

/**
 * @brief Publishes the state machine data if validation is successful.
 * @exception std::exception Thrown if the state machine fails validation.
 */
void YasminViewerPub::publish_data() {

  try {
    this->fsm->validate();

    std::vector<yasmin_msgs::msg::State> states_list;
    this->parse_state(this->fsm_name, this->fsm, {}, states_list, -1);

    auto state_machine_msg = yasmin_msgs::msg::StateMachine();
    state_machine_msg.states = states_list;

    YASMIN_LOG_DEBUG("Publishing data of state machine '%s'",
                     this->fsm_name.c_str());
    this->publisher->publish(state_machine_msg);

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR(
        "Not publishing state machine '%s' due to validation failure: \"%s\"",
        this->fsm_name.c_str(), std::string(e.what()).c_str());
  }
}
