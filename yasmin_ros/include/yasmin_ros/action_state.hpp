
#ifndef YASMIN_ROS_ACTION_STATE_HPP
#define YASMIN_ROS_ACTION_STATE_HPP

#include <memory>
#include <string>

#include "simple_node/actions/action_client.hpp"
#include "simple_node/node.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {

template <typename ActionT> class AcionState : public yasmin::State {

  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using CreateGoalHandler =
      std::function<Goal(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  using ResutlHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Result)>;

public:
  AcionState(simple_node::Node *node, std::string action_name,
             CreateGoalHandler create_goal_handler,
             std::vector<std::string> outcomes)
      : AcionState(node, action_name, create_goal_handler, outcomes, nullptr) {}

  AcionState(simple_node::Node *node, std::string action_name,
             CreateGoalHandler create_goal_handler,
             std::vector<std::string> outcomes, ResutlHandler result_handler)
      : State({}) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT,
                      basic_outcomes::CANCEL};

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.push_back(outcome);
      }
    }

    this->action_client = node->create_action_client<ActionT>(action_name);

    this->create_goal_handler = create_goal_handler;
    this->result_handler = result_handler;

    if (this->create_goal_handler == nullptr) {
      throw std::invalid_argument("create_goal_handler is needed");
    }
  }

  void cancel_state() override {
    this->action_client->cancel_goal();
    yasmin::State::cancel_state();
  }

  std::string execute(yasmin::blackboard::Blackboard blackboard) {

    Goal goal = this->create_goal(blackboard);

    this->action_client->wait_for_server();
    this->action_client->send_goal(goal);
    this->action_client->wait_for_result();

    if (this->action_client->is_canceled()) {
      return basic_outcomes::CANCEL;

    } else if (this->action_client->is_aborted()) {
      return basic_outcomes::ABORT;

    } else if (this->action_client->is_succeeded()) {
      Result result = this->action_client.get_result();

      if (this->result_handler != nullptr) {
        std::string outcome = this->result_handler(blackboard, result);
        return outcome;
      }

      return basic_outcomes::SUCCEED;
    }
  }

private:
  std::shared_ptr<simple_node::actions::ActionClient<ActionT>> action_client;
  CreateGoalHandler create_goal_handler;
  ResutlHandler result_handler;

  Goal create_goal(yasmin::blackboard::Blackboard blackboard) {
    return this->create_goal_handler(blackboard);
  }
};

} // namespace yasmin_ros

#endif
