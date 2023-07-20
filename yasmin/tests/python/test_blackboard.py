# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import unittest
from yasmin import Blackboard


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
