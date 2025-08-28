# Copyright (C) 2023 Miguel Ángel González Santamarta

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

    def test_get(self):
        self.blackboard["foo"] = "foo"
        self.assertEqual("foo", self.blackboard["foo"])

    def test_delete(self):
        self.blackboard["foo"] = "foo"
        del self.blackboard["foo"]
        self.assertFalse("foo" in self.blackboard)

    def test_contains(self):
        self.blackboard["foo"] = "foo"
        self.assertTrue("foo" in self.blackboard)

    def test_len(self):
        self.blackboard["foo"] = "foo"
        self.assertEqual(1, len(self.blackboard))
