
#include <iostream>
#include <memory>
#include <string>

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

// define state Foo
class FooState : public yasmin::State {
public:
  int counter;

  FooState() : yasmin::State({"outcome1", "outcome2"}) { this->counter = 0; };

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state FOO\n";
    if (this->counter < 3) {
      this->counter += 1;
      blackboard->set<std::string>("foo_str", "Counter: " + this->counter);
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
    std::cout << blackboard->get<std::string>("foo_str") << "\n";

    return "outcome2";
  }

  std::string to_string() { return "BarState"; }
};

int main() {

  std::cout << "yasmin_demo\n";

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::StateMachine({"outcome4", "outcome5"}));

  // add states
  sm->add_state("FOO", std::make_shared<FooState>(),
                {{"outcome1", "BAR"}, {"outcome2", "outcome4"}});
  sm->add_state("BAR", std::make_shared<BarState>(), {{"outcome2", "FOO"}});

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  return 0;
}
