import unittest
from yasmin import State


class FooState(State):
    def __init__(self):
        super().__init__(["outcome1"])

    def execute(self, blackboard):
        return "outcome1"


class BarState(State):
    def __init__(self):
        super().__init__([])

    def execute(self, blackboard):
        return "outcome2"


class TestState(unittest.TestCase):

    def setUp(self):

        self.state = FooState()

    def test_state_call(self):
        self.assertEqual("outcome1", self.state())

    def test_state_cancel(self):
        self.assertFalse(self.state.is_canceled())
        self.state.cancel_state()
        self.assertTrue(self.state.is_canceled())

    def test_state_get_outcomes(self):
        self.assertEqual("outcome1", self.state.get_outcomes()[0])

    def test_state_str(self):
        self.assertEqual("FooState", str(self.state))

    def test_state_init_exception(self):
        self.assertRaises(Exception, BarState)
