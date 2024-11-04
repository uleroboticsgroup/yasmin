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

    def test_call(self):
        self.assertEqual("outcome1", self.state())

    def test_cancel(self):
        self.assertFalse(self.state.is_canceled())
        self.state.cancel_state()
        self.assertTrue(self.state.is_canceled())

    def test_get_outcomes(self):
        self.assertEqual("outcome1", list(self.state.get_outcomes())[0])

    def test_str(self):
        self.assertEqual("FooState", str(self.state))

    def test_init_exception(self):
        self.assertRaises(Exception, BarState)
