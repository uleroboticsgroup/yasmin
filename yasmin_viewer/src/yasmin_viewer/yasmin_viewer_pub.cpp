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

#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin_viewer;
using namespace std::chrono_literals;

YasminViewerPub::YasminViewerPub(rclcpp::Node *node, std::string fsm_name,
                                 std::shared_ptr<yasmin::StateMachine> fsm) {

  this->node = node;

  this->thread = std::make_unique<std::thread>(
      std::thread(&YasminViewerPub::start_publisher, this, fsm_name, fsm));
  this->thread->detach();
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

void YasminViewerPub::start_publisher(
    std::string fsm_name, std::shared_ptr<yasmin::StateMachine> fsm) {

  auto publisher = this->node->create_publisher<yasmin_msgs::msg::StateMachine>(
      "/fsm_viewer", 10);

  while (rclcpp::ok()) {
    std::vector<yasmin_msgs::msg::State> states_list;
    this->parse_state(fsm_name, fsm, {}, states_list, -1);

    auto state_machine_msg = yasmin_msgs::msg::StateMachine();
    state_machine_msg.states = states_list;

    publisher->publish(state_machine_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}
