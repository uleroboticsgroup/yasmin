#!/usr/bin/env python3

from yasmin import State
from yasmin import StateMachine


# define state Foo
class FooState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state FOO")
        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = "Counter: " + str(self.counter)
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["outcome2"])

    def execute(self, blackboard):
        print("Executing state BAR")
        print(blackboard.foo_str)
        return "outcome2"


# main
def main():
    print("yasmin_demo")

    # create a state machine
    sm = StateMachine(outcomes=["outcome4", "outcome5"])

    # add states
    sm.add_state("FOO", FooState(),
                 transitions={"outcome1": "BAR",
                              "outcome2": "outcome4"})
    sm.add_state("BAR", BarState(),
                 transitions={"outcome2": "FOO"})

    # rxecute
    outcome = sm()
    print(outcome)


if __name__ == "__main__":
    main()
