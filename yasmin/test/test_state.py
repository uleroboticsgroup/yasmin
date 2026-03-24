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
        self.add_input_key("counter", "An integer counter", 42)
        self.add_input_key("label", "A string label", "hello")

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
        state.add_input_key("speed", "Robot linear speed")
        keys = state.get_input_keys()
        speed_key = next(k for k in keys if k["name"] == "speed")
        self.assertEqual(speed_key["description"], "Robot linear speed")
        self.assertFalse(speed_key["has_default"])

    def test_key_description_empty_by_default(self):
        state = FooState()
        state.add_input_key("raw_key")
        keys = state.get_input_keys()
        self.assertEqual(keys[0]["description"], "")


class StateWithAllTypes(State):
    def __init__(self):
        super().__init__(["done"])
        self.set_description("State with various default types")
        self.set_outcome_description("done", "Main outcome")
        self.add_input_key("flag", "A boolean flag", True)
        self.add_input_key("speed", "Speed value", 3.14)
        self.add_input_key("count", "An integer count", 10)
        self.add_input_key("name", "Name string", "robot")
        self.add_output_key("result", "Result value")

    def execute(self, blackboard):
        return "done"


class TestStateMetadataExtended(unittest.TestCase):

    def test_description_set_and_get(self):
        state = StateWithAllTypes()
        self.assertEqual(state.get_description(), "State with various default types")

    def test_description_default_empty(self):
        state = FooState()
        self.assertEqual(state.get_description(), "")

    def test_outcome_description(self):
        state = StateWithAllTypes()
        self.assertEqual(state.get_outcome_description("done"), "Main outcome")

    def test_multiple_input_key_types(self):
        state = StateWithAllTypes()
        keys = state.get_input_keys()
        self.assertEqual(len(keys), 4)

        flag_key = next(k for k in keys if k["name"] == "flag")
        self.assertTrue(flag_key["has_default"])
        self.assertEqual(flag_key["default_value"], True)
        self.assertIsInstance(flag_key["default_value"], bool)

        speed_key = next(k for k in keys if k["name"] == "speed")
        self.assertTrue(speed_key["has_default"])
        self.assertAlmostEqual(speed_key["default_value"], 3.14, places=2)

        count_key = next(k for k in keys if k["name"] == "count")
        self.assertTrue(count_key["has_default"])
        self.assertEqual(count_key["default_value"], 10)

        name_key = next(k for k in keys if k["name"] == "name")
        self.assertTrue(name_key["has_default"])
        self.assertEqual(name_key["default_value"], "robot")

    def test_output_key(self):
        state = StateWithAllTypes()
        keys = state.get_output_keys()
        self.assertEqual(len(keys), 1)

        result_key = next(k for k in keys if k["name"] == "result")
        # Output keys are not allowed to have defaults
        self.assertFalse(result_key["has_default"])

    def test_all_defaults_injected(self):
        state = StateWithAllTypes()
        bb = Blackboard()
        state(bb)
        self.assertEqual(bb["flag"], True)
        self.assertAlmostEqual(bb["speed"], 3.14, places=2)
        self.assertEqual(bb["count"], 10)
        self.assertEqual(bb["name"], "robot")

    def test_get_metadata_complete(self):
        state = StateWithAllTypes()
        meta = state.get_metadata()
        self.assertEqual(meta["description"], "State with various default types")
        self.assertEqual(len(meta["input_keys"]), 4)
        self.assertEqual(len(meta["output_keys"]), 1)

    def test_add_input_key_with_list_default(self):
        state = FooState()
        state.add_input_key("items", "A list", [1, 2, 3])
        keys = state.get_input_keys()
        item_key = next(k for k in keys if k["name"] == "items")
        self.assertTrue(item_key["has_default"])
        self.assertEqual(item_key["default_value"], [1, 2, 3])

    def test_add_input_key_with_dict_default(self):
        state = FooState()
        state.add_input_key("config", "A dict", {"a": 1})
        keys = state.get_input_keys()
        config_key = next(k for k in keys if k["name"] == "config")
        self.assertTrue(config_key["has_default"])
        self.assertEqual(config_key["default_value"], {"a": 1})

    def test_default_injection_with_complex_type(self):
        state = FooState()
        state.add_input_key("items", "A list", [1, 2, 3])
        bb = Blackboard()
        state(bb)
        self.assertEqual(bb["items"], [1, 2, 3])


if __name__ == "__main__":
    unittest.main()
