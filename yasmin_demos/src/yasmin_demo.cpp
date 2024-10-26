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
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

// define state Foo
class FooState : public yasmin::State {
public:
  int counter;

  FooState() : yasmin::State({"outcome1", "outcome2"}) { this->counter = 0; };

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state FOO\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (this->counter < 3) {
      this->counter += 1;
      blackboard->set<std::string>("foo_str",
                                   "Counter: " + std::to_string(this->counter));
      return "outcome1";

    } else {
      return "outcome2";
    }
  }

  std::string to_string() { return "FooState"; }
};

// define state Bar
class BarState : public yasmin::State {
public:
  BarState() : yasmin::State({"outcome3"}){};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state BAR\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << blackboard->get<std::string>("foo_str") << "\n";

    return "outcome3";
  }

  std::string to_string() { return "BarState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // add states
  sm->add_state("FOO", std::make_shared<FooState>(),
                {{"outcome1", "BAR"}, {"outcome2", "outcome4"}});
  sm->add_state("BAR", std::make_shared<BarState>(), {{"outcome3", "FOO"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("yasmin_demo", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
