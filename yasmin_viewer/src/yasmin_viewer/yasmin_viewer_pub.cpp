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

#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/types.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_viewer;
using namespace std::chrono_literals;

YasminViewerPub::YasminViewerPub(yasmin::StateMachine::SharedPtr fsm)
    : YasminViewerPub(nullptr, fsm, "") {}

YasminViewerPub::YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                                 yasmin::StateMachine::SharedPtr fsm)
    : YasminViewerPub(node, fsm, "") {}

YasminViewerPub::YasminViewerPub(yasmin::StateMachine::SharedPtr fsm,
                                 const std::string &fsm_name)
    : YasminViewerPub(nullptr, fsm, fsm_name) {}

YasminViewerPub::YasminViewerPub(const rclcpp::Node::SharedPtr &node,
                                 yasmin::StateMachine::SharedPtr fsm,
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

std::vector<yasmin_msgs::msg::Transition> YasminViewerPub::parse_transitions(
    const yasmin::Transitions &transitions) const {
  std::vector<yasmin_msgs::msg::Transition> transitions_list;
  transitions_list.reserve(transitions.size());

  for (const auto &transition : transitions) {
    auto &transition_msg = transitions_list.emplace_back();
    transition_msg.outcome = transition.first;
    transition_msg.state = transition.second;
  }
  return transitions_list;
}

std::unordered_map<std::string, std::vector<yasmin_msgs::msg::Transition>>
YasminViewerPub::parse_concurrence_transitions(
    yasmin::Concurrence::SharedPtr concurrence) const {
  std::unordered_map<std::string, std::vector<yasmin_msgs::msg::Transition>>
      transitions;
  const auto &outcome_map = concurrence->get_outcome_map();

  // Add transitions for each outcome in the outcome map
  for (const auto &[outcome, requirements] : outcome_map) {
    for (const auto &[state_name, state_outcome] : requirements) {
      auto &transition_msg = transitions[state_name].emplace_back();
      transition_msg.outcome = state_outcome;
      transition_msg.state = outcome;
    }
  }

  return transitions;
}

void YasminViewerPub::parse_state(
    const std::string &state_name, yasmin::State::SharedPtr state,
    const yasmin::Transitions &transitions,
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

  // Check if state is a nested FSM, Concurrence, or OrthogonalState
  auto fsm = std::dynamic_pointer_cast<yasmin::StateMachine>(state);
  auto concurrence = std::dynamic_pointer_cast<yasmin::Concurrence>(state);
  auto orthogonal = std::dynamic_pointer_cast<yasmin::OrthogonalState>(state);
  state_msg.is_fsm =
      (fsm != nullptr) || (concurrence != nullptr) || (orthogonal != nullptr);

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
      yasmin::Transitions empty_transitions;
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
  // Handle orthogonal states
  else if (orthogonal != nullptr) {
    const auto &regions = orthogonal->get_regions();
    states_list[state_msg.id].current_state = -2;

    for (const auto &region : regions) {
      // Create a synthetic container node for the region
      auto region_msg = yasmin_msgs::msg::State();
      region_msg.id = states_list.size();
      region_msg.parent = state_msg.id;
      region_msg.name = region.name;
      region_msg.is_fsm = true;
      region_msg.current_state = -1;
      auto region_outcomes = region.sm->get_outcomes();
      region_msg.outcomes = std::vector<std::string>(region_outcomes.begin(),
                                                     region_outcomes.end());
      states_list.push_back(region_msg);

      auto region_states = region.sm->get_states();
      auto region_transitions = region.sm->get_transitions();

      for (const auto &nested_state : region_states) {
        yasmin::Transitions transitions;
        auto trans_it = region_transitions.find(nested_state.first);
        if (trans_it != region_transitions.end()) {
          transitions = trans_it->second;
        }

        this->parse_state(nested_state.first, nested_state.second, transitions,
                          states_list, region_msg.id);
      }

      std::string region_current = region.sm->get_current_state();
      if (!region_current.empty()) {
        for (auto &child : states_list) {
          if (child.name == region_current && child.parent == region_msg.id) {
            states_list[region_msg.id].current_state = child.id;
            break;
          }
        }
      }
    }
  }
}

void YasminViewerPub::publish_data() {

  if (!rclcpp::ok()) {
    return;
  }

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
