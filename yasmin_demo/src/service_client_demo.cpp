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

#include "example_interfaces/srv/add_two_ints.hpp"
#include "simple_node/node.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

std::string
set_ints(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("a", 10);
  blackboard->set<int>("b", 5);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

std::string
print_sum(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  fprintf(stderr, "Sum: %d\n", blackboard->get<int>("sum"));
  return yasmin_ros::basic_outcomes::SUCCEED;
}

class AddTwoIntsState
    : public yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> {

public:
  AddTwoIntsState(simple_node::Node *node)
      : yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> // msg
        (node,                                                        // node
         "/add_two_ints", // topic name
         std::bind(&AddTwoIntsState::create_request_handler, this, _1),
         {"outcome1"},
         std::bind(&AddTwoIntsState::response_handler, this, _1, _2)){};

  example_interfaces::srv::AddTwoInts::Request::SharedPtr
  create_request_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    request->a = blackboard->get<int>("a");
    request->b = blackboard->get<int>("b");

    return request;
  }

  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {

    blackboard->set<int>("sum", response->sum);

    return "outcome1";
  }

  std::string to_string() { return "AddTwoIntsState"; }
};

class ServiceClientDemoNode : public simple_node::Node {
public:
  std::unique_ptr<yasmin_viewer::YasminViewerPub> yamin_pub;

  ServiceClientDemoNode() : simple_node::Node("yasmin_node") {

    // create a state machine
    auto sm = std::make_shared<yasmin::StateMachine>(
        yasmin::StateMachine({"outcome4"}));

    // add states
    sm->add_state("SETTING_INTS",
                  std::make_shared<yasmin::CbState>(yasmin::CbState(
                      {yasmin_ros::basic_outcomes::SUCCEED}, set_ints)),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "ADD_TWO_INTS"}});
    sm->add_state("ADD_TWO_INTS", std::make_shared<AddTwoIntsState>(this),
                  {{"outcome1", "PRINTING_SUM"},
                   {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                   {yasmin_ros::basic_outcomes::ABORT, "outcome4"}});
    sm->add_state("PRINTING_SUM",
                  std::make_shared<yasmin::CbState>(yasmin::CbState(
                      {yasmin_ros::basic_outcomes::SUCCEED}, print_sum)),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "outcome4"}});

    // pub
    this->yamin_pub = std::make_unique<yasmin_viewer::YasminViewerPub>(
        yasmin_viewer::YasminViewerPub(this, "YASMIN_SERVICE_CLIENT_DEMO", sm));

    // execute
    std::string outcome = (*sm.get())();
    std::cout << outcome << "\n";
  }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_service_client_demo\n";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceClientDemoNode>();
  node->join_spin();
  rclcpp::shutdown();

  return 0;
}
