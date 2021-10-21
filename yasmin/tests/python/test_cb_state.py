import unittest
from yasmin import CbState


class TestCbState(unittest.TestCase):

    def setUp(self):

        def execute(blackboard):
            return "outcome1"

        self.state = CbState(["outcome1"], execute)

    def test_cb_state_call(self):
        self.assertEqual("outcome1", self.state())
