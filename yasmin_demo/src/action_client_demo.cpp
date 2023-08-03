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

#include <iostream>
#include <memory>
#include <string>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "simple_node/node.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

std::string
set_int(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("n", 3);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

std::string
print_result(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  auto fibo_res = blackboard->get<std::vector<int>>("sum");

  fprintf(stderr, "Sum:");

  for (auto ele : fibo_res) {
    fprintf(stderr, " %d,", ele);
  }

  fprintf(stderr, "\n");

  return yasmin_ros::basic_outcomes::SUCCEED;
}

class FibonacciState : public yasmin_ros::ActionState<
                           action_tutorials_interfaces::action::Fibonacci> {

public:
  FibonacciState(simple_node::Node *node)
      : yasmin_ros::ActionState<
            action_tutorials_interfaces::action::Fibonacci> // msg
        (node,                                              // node
         "/fibonacci",                                      // action name
         std::bind(&FibonacciState::create_goal_handler, this, _1),
         std::bind(&FibonacciState::response_handler, this, _1, _2)){};

  action_tutorials_interfaces::action::Fibonacci::Goal::SharedPtr
  create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = std::make_shared<
        action_tutorials_interfaces::action::Fibonacci::Goal>();

    goal->order = blackboard->get<int>("n");

    return goal;
  }

  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      action_tutorials_interfaces::action::Fibonacci::Result::SharedPtr
          response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  std::string to_string() { return "FibonacciState"; }
};

class ActionClientDemoNode : public simple_node::Node {
public:
  std::unique_ptr<yasmin_viewer::YasminViewerPub> yamin_pub;

  ActionClientDemoNode() : simple_node::Node("yasmin_node") {

    // create a state machine
    auto sm = std::make_shared<yasmin::StateMachine>(
        yasmin::StateMachine({"outcome4"}));

    // add states
    sm->add_state("SETTING_INT",
                  std::make_shared<yasmin::CbState>(yasmin::CbState(
                      {yasmin_ros::basic_outcomes::SUCCEED}, set_int)),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "CALLING_FIBONACCI"}});
    sm->add_state("CALLING_FIBONACCI", std::make_shared<FibonacciState>(this),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "PRINTING_RESULT"},
                   {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                   {yasmin_ros::basic_outcomes::ABORT, "outcome4"}});
    sm->add_state("PRINTING_RESULT",
                  std::make_shared<yasmin::CbState>(yasmin::CbState(
                      {yasmin_ros::basic_outcomes::SUCCEED}, print_result)),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "outcome4"}});

    // pub
    this->yamin_pub = std::make_unique<yasmin_viewer::YasminViewerPub>(
        yasmin_viewer::YasminViewerPub(this, "YASMIN_ACTION_CLIENT_DEMO", sm));

    // execute
    std::string outcome = (*sm.get())();
    std::cout << outcome << "\n";
  }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_action_client_demo\n";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionClientDemoNode>();
  node->join_spin();
  rclcpp::shutdown();

  return 0;
}
