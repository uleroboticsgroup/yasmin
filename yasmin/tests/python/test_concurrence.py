# Copyright (C) 2025 Georgia Tech Research Institute
# Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
#
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
import time
from yasmin import State, Concurrence


class FooState(State):
    def __init__(self):
        super().__init__(["outcome1"])

    def execute(self, blackboard):
        time.sleep(0.1)
        print("Foo state ticked.")
        time.sleep(0.2)
        print("Foo state ended.")

        return "outcome1"


class BarState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])

    def execute(self, blackboard):
        time.sleep(0.2)
        print("Bar state ticked.")
        time.sleep(0.1)
        print("Bar state ended.")

        return "outcome2"


class TestState(unittest.TestCase):

    def setUp(self):
        self.foo_state = FooState()
        self.foo2_state = FooState()
        self.bar_state = BarState()
        self.state = Concurrence(
            states=[self.foo_state, self.foo2_state, self.bar_state],
            default_outcome="default",
            outcome_map={
                "outcome1": {self.foo_state: "outcome1"},
                "outcome2": {self.bar_state: "outcome1", self.bar_state: "outcome1"},
            },
        )

    def instance_exception(self):
        self.concurrent_state = Concurrence(
            states=[self.foo_state, self.foo_state], default_outcome="foo"
        )

    def key_exception(self):
        self.concurrent_state = Concurrence(
            states=[self.foo_state],
            outcome_map={"outcome1": {self.foo_state: "non_outcome"}},
            default_outcome="foo",
        )

    def test_call(self):
        self.assertEqual("outcome1", self.state())

    def test_cancel(self):
        self.assertFalse(self.state.is_canceled())
        self.state.cancel_state()
        self.assertTrue(self.state.is_canceled())

    def test_str(self):
        self.assertEqual("Concurrence [FooState, FooState, BarState]", str(self.state))

    def test_instance_exception(self):
        self.assertRaises(Exception, self.instance_exception)

    def test_key_exception(self):
        self.assertRaises(Exception, self.key_exception)
