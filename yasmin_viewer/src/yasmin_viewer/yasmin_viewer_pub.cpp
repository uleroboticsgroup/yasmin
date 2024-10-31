// Copyright (C) 2023  Miguel Ángel González Santamarta

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

#include <chrono>

#include "yasmin/logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin_viewer;
using namespace std::chrono_literals;
using namespace yasmin;

YasminViewerPub::YasminViewerPub(std::string fsm_name,
                                 std::shared_ptr<yasmin::StateMachine> fsm)
    : YasminViewerPub(nullptr, fsm_name, fsm) {}

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

void YasminViewerPub::parse_state(
    std::string state_name, std::shared_ptr<yasmin::State> state,
    std::map<std::string, std::string> transitions,
    std::vector<yasmin_msgs::msg::State> &states_list, int parent) {

  auto state_msg = yasmin_msgs::msg::State();

  state_msg.id = states_list.size();
  state_msg.parent = parent;

  // state info
  state_msg.name = state_name;
  state_msg.transitions = this->parse_transitions(transitions);
  state_msg.outcomes = state->get_outcomes();

  // state is a FSM
  auto fsm = std::dynamic_pointer_cast<yasmin::StateMachine>(state);
  state_msg.is_fsm = (fsm != nullptr);

  // add state
  states_list.push_back(state_msg);

  // states of the FSM
  if (state_msg.is_fsm) {
    auto states = fsm->get_states();
    auto aux_transitions = fsm->get_transitions();

    for (auto const &state : states) {
      this->parse_state(state.first, state.second, aux_transitions[state.first],
                        states_list, state_msg.id);
    }

    std::string current_state = fsm->get_current_state();
    for (auto const &child_state : states_list) {

      if (child_state.name == current_state &&
          child_state.parent == state_msg.id) {
        states_list[state_msg.id].current_state = child_state.id;
        break;
      }
    }
  }
}

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
    YASMIN_LOG_ERROR("Not publishing state machine '%s' due to "
                     "validation failure: \"%s\"",
                     this->fsm_name.c_str(), e.what());
  }
}
