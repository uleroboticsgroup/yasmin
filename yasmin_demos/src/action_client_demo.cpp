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

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using namespace yasmin;

std::string
print_result(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  auto fibo_res = blackboard->get<std::vector<int>>("sum");

  fprintf(stderr, "Result received:");

  for (auto ele : fibo_res) {
    fprintf(stderr, " %d,", ele);
  }

  fprintf(stderr, "\n");

  return yasmin_ros::basic_outcomes::SUCCEED;
}

class FibonacciState : public yasmin_ros::ActionState<Fibonacci> {

public:
  FibonacciState()
      : yasmin_ros::ActionState<Fibonacci>(

            "/fibonacci", // action name

            // # cb to create the goal
            std::bind(&FibonacciState::create_goal_handler, this, _1),
            // # cb to process the response

            std::bind(&FibonacciState::response_handler, this, _1, _2),

            // cb to process the feedback
            std::bind(&FibonacciState::print_feedback, this, _1, _2)){};

  Fibonacci::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = Fibonacci::Goal();
    goal.order = blackboard->get<int>("n");

    return goal;
  }

  std::string
  response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                   Fibonacci::Result::SharedPtr response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void
  print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 std::shared_ptr<const Fibonacci::Feedback> feedback) {
    (void)blackboard;

    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }

    fprintf(stderr, "%s\n", ss.str().c_str());
  }
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_action_client_demo");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("CALLING_FIBONACCI", std::make_shared<FibonacciState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "PRINTING_RESULT"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("PRINTING_RESULT",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    print_result),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // create an initial blackboard
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("n", 10);

  // execute
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
