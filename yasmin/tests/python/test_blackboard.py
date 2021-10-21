import unittest
from yasmin.blackboard import Blackboard


class TestBlackboard(unittest.TestCase):

    def setUp(self):

        self.blackboard = Blackboard()

    def test_blackboard_get(self):
        self.blackboard.__setitem__("foo", "foo")
        self.assertEqual("foo", self.blackboard.__getitem__("foo"))

    def test_blackboard_delete(self):
        self.blackboard.__setitem__("foo", "foo")
        self.blackboard.__delitem__("foo")
        self.assertFalse(self.blackboard.__contains__("foo"))

    def test_blackboard_contains(self):
        self.blackboard.__setitem__("foo", "foo")
        self.assertTrue(self.blackboard.__contains__("foo"))

    def test_blackboard_len(self):
        self.blackboard.__setitem__("foo", "foo")
        self.assertEqual(1, self.blackboard.__len__())
