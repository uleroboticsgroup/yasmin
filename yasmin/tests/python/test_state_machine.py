import unittest
from yasmin import StateMachine, State

# define state Foo


class FooState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard):
        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = "Counter: " + str(self.counter)
            return "outcome1"
        else:
            return "outcome2"


class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["outcome2"])

    def execute(self, blackboard):
        return "outcome2"


class TestStateMachine(unittest.TestCase):

    def setUp(self):

        self.sm = StateMachine(outcomes=["outcome4", "outcome5"])

        self.sm.add_state("FOO", FooState(),
                          transitions={"outcome1": "BAR",
                                       "outcome2": "outcome4"})
        self.sm.add_state("BAR", BarState(),
                          transitions={"outcome2": "FOO"})

    def test_state_machine_get_states(self):

        self.assertTrue(isinstance(self.sm.get_states()
                                   ["FOO"]["state"], FooState))
        self.assertTrue(isinstance(self.sm.get_states()
                                   ["BAR"]["state"], BarState))

    def test_state_machine_get_start_state(self):

        self.assertEqual("FOO", self.sm.get_start_state())
        self.sm.set_start_state("BAR")
        self.assertEqual("BAR", self.sm.get_start_state())

    def test_state_machine_get_current_state(self):

        self.assertEqual("", self.sm.get_current_state())

    def test_state_call(self):

        self.assertEqual("outcome4", self.sm())
