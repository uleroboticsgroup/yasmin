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

yasmin_interfaces::msg::StateInfo YasminViewerPub::parse_state_info(
    std::string state_name, std::shared_ptr<yasmin::State> state,
    std::map<std::string, std::string> transitions) {

  auto state_info_msg = yasmin_interfaces::msg::StateInfo();

  for (auto const &transition : transitions) {
    auto transition_msg = yasmin_interfaces::msg::Transition();
    transition_msg.outcome = transition.first;
    transition_msg.state = transition.second;
    state_info_msg.transitions.push_back(transition_msg);
  }

  for (std::string outcome : state->get_outcomes()) {
    state_info_msg.outcomes.push_back(outcome);
  }

  state_info_msg.state_name = state_name;

  return state_info_msg;
}

yasmin_interfaces::msg::Structure
YasminViewerPub::parse_states(std::shared_ptr<yasmin::StateMachine> fsm) {

  std::map<std::string, std::shared_ptr<yasmin::State>> states =
      fsm->get_states();
  auto structure_msg = yasmin_interfaces::msg::Structure();

  for (auto const &state : states) {

    auto state_msg = yasmin_interfaces::msg::State();
    auto state_info = this->parse_state_info(
        state.first, state.second, fsm->get_transitions().at(state.first));
    state_msg.state = state_info;

    // check if state is a fsm
    auto state_machine =
        std::dynamic_pointer_cast<yasmin::StateMachine>(state.second);

    if (state_machine != nullptr) {

      auto aux_structure_msg = this->parse_states(state_machine);
      state_msg.is_fsm = true;

      for (yasmin_interfaces::msg::State aux_state_msg :
           aux_structure_msg.states) {
        state_msg.states.push_back(aux_state_msg.state);
      }

      state_msg.current_state = state_machine->get_current_state();
    }

    structure_msg.states.push_back(state_msg);
  }

  return structure_msg;
}

yasmin_interfaces::msg::Status
YasminViewerPub::parse_status(std::shared_ptr<yasmin::StateMachine> fsm) {

  auto status_msg = yasmin_interfaces::msg::Status();

  auto structure_msg = this->parse_states(fsm);
  structure_msg.final_outcomes = fsm->get_outcomes();

  status_msg.fsm_structure = structure_msg;
  status_msg.current_state = fsm->get_current_state();

  return status_msg;
}

void YasminViewerPub::start_publisher(
    std::string fsm_name, std::shared_ptr<yasmin::StateMachine> fsm) {

  auto publisher = this->node->create_publisher<yasmin_interfaces::msg::Status>(
      "/fsm_viewer", 10);

  while (rclcpp::ok()) {

    auto status_msg = this->parse_status(fsm);
    status_msg.fsm_name = fsm_name;

    publisher->publish(status_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}
