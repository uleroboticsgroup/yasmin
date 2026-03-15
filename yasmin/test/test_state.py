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
from yasmin import Blackboard, State


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


class StateWithDefaults(State):
    def __init__(self):
        super().__init__(["done"])
        self.add_input_key("counter", 42, "An integer counter")
        self.add_input_key("label", "hello", "A string label")

    def execute(self, blackboard):
        return "done"


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
        with self.assertRaises(RuntimeError) as context:
            BarState()
        self.assertEqual(
            str(context.exception), "A state must have at least one possible outcome."
        )


class TestStateMetadata(unittest.TestCase):

    def test_default_value_injected_when_missing(self):
        state = StateWithDefaults()
        bb = Blackboard()
        state(bb)
        self.assertEqual(bb["counter"], 42)
        self.assertEqual(bb["label"], "hello")

    def test_default_value_not_overwritten_when_present(self):
        state = StateWithDefaults()
        bb = Blackboard()
        bb["counter"] = 100
        state(bb)
        self.assertEqual(bb["counter"], 100)

    def test_default_value_respects_mapped_key_exists(self):
        state = StateWithDefaults()
        bb = Blackboard()
        bb.set_remappings({"counter": "my_counter"})
        bb["my_counter"] = 99
        state(bb)
        self.assertEqual(bb["my_counter"], 99)

    def test_default_value_injected_on_remapped_key(self):
        state = StateWithDefaults()
        bb = Blackboard()
        bb.set_remappings({"counter": "my_counter"})
        state(bb)
        self.assertEqual(bb["my_counter"], 42)

    def test_default_value_each_call_is_independent(self):
        state = StateWithDefaults()
        bb1 = Blackboard()
        bb2 = Blackboard()
        state(bb1)
        bb1["counter"] = 7
        state(bb2)
        self.assertEqual(bb2["counter"], 42)

    def test_key_description_field(self):
        state = StateWithDefaults()
        keys = state.get_input_keys()
        counter_key = next(k for k in keys if k["name"] == "counter")
        self.assertEqual(counter_key["description"], "An integer counter")
        label_key = next(k for k in keys if k["name"] == "label")
        self.assertEqual(label_key["description"], "A string label")

    def test_add_input_key_description_no_default(self):
        state = StateWithDefaults()
        state.add_input_key("speed", description="Robot linear speed")
        keys = state.get_input_keys()
        speed_key = next(k for k in keys if k["name"] == "speed")
        self.assertEqual(speed_key["description"], "Robot linear speed")
        self.assertFalse(speed_key["has_default"])

    def test_key_description_empty_by_default(self):
        state = FooState()
        state.add_input_key("raw_key")
        keys = state.get_input_keys()
        self.assertEqual(keys[0]["description"], "")


if __name__ == "__main__":
    unittest.main()
