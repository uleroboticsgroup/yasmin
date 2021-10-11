
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "simple_node/node.hpp"

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
  BarState() : yasmin::State({"outcome2"}){};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state BAR\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << blackboard->get<std::string>("foo_str") << "\n";

    return "outcome2";
  }

  std::string to_string() { return "BarState"; }
};

class DemoNode : public simple_node::Node {
public:
  std::unique_ptr<yasmin_viewer::YasminViewerPub> yamin_pub;

  DemoNode() : simple_node::Node("yasmin_node") {

    // create a state machine
    auto sm = std::make_shared<yasmin::StateMachine>(
        yasmin::StateMachine({"outcome4", "outcome5"}));

    // add states
    sm->add_state("FOO", std::make_shared<FooState>(),
                  {{"outcome1", "BAR"}, {"outcome2", "outcome4"}});
    sm->add_state("BAR", std::make_shared<BarState>(), {{"outcome2", "FOO"}});

    // pub
    this->yamin_pub = std::make_unique<yasmin_viewer::YasminViewerPub>(
        yasmin_viewer::YasminViewerPub(this, "YASMIN_DEMO", sm));

    // execute
    std::string outcome = (*sm.get())();
    std::cout << outcome << "\n";
  }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_demo\n";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();
  node->join_spin();
  rclcpp::shutdown();

  return 0;
}
