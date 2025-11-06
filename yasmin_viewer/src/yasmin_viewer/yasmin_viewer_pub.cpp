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

#include "yasmin_viewer/yasmin_viewer_pub.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_viewer;
using namespace std::chrono_literals;

YasminViewerPub::YasminViewerPub(std::shared_ptr<yasmin::StateMachine> fsm)
    : YasminViewerPub(nullptr, fsm, "") {}

YasminViewerPub::YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                                 std::shared_ptr<yasmin::StateMachine> fsm)
    : YasminViewerPub(node, fsm, "") {}

YasminViewerPub::YasminViewerPub(std::shared_ptr<yasmin::StateMachine> fsm,
                                 const std::string &fsm_name)
    : YasminViewerPub(nullptr, fsm, fsm_name) {}

YasminViewerPub::YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                                 std::shared_ptr<yasmin::StateMachine> fsm,
                                 const std::string &fsm_name)
    : fsm(fsm), fsm_name(fsm_name) {

  if (node == nullptr) {
    this->node_ = yasmin_ros::YasminNode::get_instance();
  } else {
    this->node_ = node;
  }

  if (this->fsm_name.empty() && this->fsm->get_name().empty()) {
    this->fsm_name = "Unnamed_FSM";
  } else if (this->fsm_name.empty()) {
    this->fsm_name = this->fsm->get_name();
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
    const std::map<std::string, std::string> &transitions) {
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
 * @brief Parses concurrence transitions from outcome map to transition-like
 * information.
 * @param concurrence Shared pointer to the Concurrence state.
 * @return Map of state names to their transition vectors.
 */
std::map<std::string, std::vector<yasmin_msgs::msg::Transition>>
YasminViewerPub::parse_concurrence_transitions(
    std::shared_ptr<yasmin::Concurrence> concurrence) {
  std::map<std::string, std::vector<yasmin_msgs::msg::Transition>> transitions;
  auto outcome_map = concurrence->get_outcome_map();

  // Add transitions for each outcome in the outcome map
  for (const auto &[outcome, requirements] : outcome_map) {
    for (const auto &[state_name, state_outcome] : requirements) {
      if (transitions.find(state_name) == transitions.end()) {
        transitions[state_name] = std::vector<yasmin_msgs::msg::Transition>();
      }
      auto transition_msg = yasmin_msgs::msg::Transition();
      transition_msg.outcome = state_outcome;
      transition_msg.state = outcome;
      transitions[state_name].push_back(transition_msg);
    }
  }

  return transitions;
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
    const std::string &state_name, std::shared_ptr<yasmin::State> state,
    const std::map<std::string, std::string> &transitions,
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

  // Check if state is a nested FSM or Concurrence
  auto fsm = std::dynamic_pointer_cast<yasmin::StateMachine>(state);
  auto concurrence = std::dynamic_pointer_cast<yasmin::Concurrence>(state);
  state_msg.is_fsm = (fsm != nullptr) || (concurrence != nullptr);

  // Add the state to the list
  states_list.push_back(state_msg);

  // Handle nested FSM states
  if (fsm != nullptr) {
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
  // Handle concurrence states
  else if (concurrence != nullptr) {
    auto concurrent_states = concurrence->get_states();
    auto transitions = this->parse_concurrence_transitions(concurrence);
    states_list[state_msg.id].current_state =
        -2; // Special marker for concurrence

    for (const auto &[child_state_name, child_state] : concurrent_states) {
      std::map<std::string, std::string> empty_transitions;
      this->parse_state(child_state_name, child_state, empty_transitions,
                        states_list, state_msg.id);

      // Set transitions for this child state
      if (transitions.find(child_state_name) != transitions.end()) {
        states_list.back().transitions = transitions[child_state_name];
      }

      // Check if the child_state outcomes are in the transitions
      auto child_outcomes = child_state->get_outcomes();
      std::set<std::string> transition_outcomes;
      for (const auto &transition : states_list.back().transitions) {
        transition_outcomes.insert(transition.outcome);
      }

      for (const auto &outcome : child_outcomes) {
        if (transition_outcomes.find(outcome) == transition_outcomes.end()) {
          // If not, add a transition to the default outcome of the concurrence
          auto msg = yasmin_msgs::msg::Transition();
          msg.outcome = outcome;
          msg.state = concurrence->get_default_outcome();
          states_list.back().transitions.push_back(msg);
        }
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
