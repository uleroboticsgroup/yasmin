# YASMIN (Yet Another State Machine)

## Simple Demo
```python
from yasmin import State
from yasmin import StateMachine


# define state Foo
class Foo(State):
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
class Bar(State):
    def __init__(self):
        super().__init__(outcomes=["outcome2"])

    def execute(self, blackboard):
        print("Executing state BAR")
        print(blackboard.foo_str)
        return "outcome2"


# main
def main():
    print("demo_state_machine")

    # Create a state machine
    sm = StateMachine(outcomes=["outcome4", "outcome5"])

    # Add states
    sm.add_state("FOO", Foo(),
                 transitions={"outcome1": "BAR",
                              "outcome2": "outcome4"})
    sm.add_state("BAR", Bar(),
                 transitions={"outcome2": "FOO"})

    # Execute
    outcome = sm()
    print(outcome)


if __name__ == "__main__":
    main()
```

## yasmin_viewer

### Installation
```shell
./install_viewer.sh 
```

### Run
```shell
ros2 run yasmin_viewer yasmin_viewer_node
```

### Viewer
http://localhost:5000/
