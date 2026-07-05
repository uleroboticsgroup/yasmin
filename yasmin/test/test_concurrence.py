# Copyright (C) 2025 Georgia Tech Research Institute
# Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import unittest
import time
from yasmin import State, Concurrence, StateMachine


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
            states={
                "FOO": self.foo_state,
                "FOO2": self.foo2_state,
                "BAR": self.bar_state,
            },
            default_outcome="default",
            outcome_map={
                "outcome1": {"FOO": "outcome1"},
                "outcome2": {"BAR": "outcome1", "BAR": "outcome1"},
            },
        )

    def test_call(self):
        self.assertEqual("outcome1", self.state())

    def test_cancel(self):
        self.assertFalse(self.state.is_canceled())
        self.state.cancel_state()
        self.assertTrue(self.state.is_canceled())

    def test_str(self):
        string = str(self.state)
        # Check if "Bar (BarState)" and "Foo (FooState)" are in the string
        self.assertIn("BAR (BarState)", string)
        self.assertIn("FOO (FooState)", string)
        self.assertIn("FOO2 (FooState)", string)


class ConfigurableConcurrentState(State):
    def __init__(self):
        super().__init__(["done"])
        self.declare_parameter("topic", "Concurrent topic")
        self.configure_count = 0
        self.configured_topic = None

    def configure(self):
        self.configure_count += 1
        self.configured_topic = self.get_parameter("topic")

    def execute(self, blackboard):
        return "done"


class TestConcurrenceConfigure(unittest.TestCase):
    def test_configure_applies_parameter_mappings_and_runs_once(self):
        child = ConfigurableConcurrentState()
        state = Concurrence(
            states={"CHILD": child},
            default_outcome="done",
            outcome_map={"done": {"CHILD": "done"}},
        )
        state.declare_parameter("topic", "Parent topic", "/concurrent")
        state.set_parameter_mappings("CHILD", {"topic": "topic"})

        self.assertEqual("done", state())
        self.assertEqual("done", state())
        self.assertEqual(1, child.configure_count)
        self.assertEqual("/concurrent", child.configured_topic)


# ---------------------------------------------------------------------------
# Validate: Concurrence recursively validates nested StateMachines
# ---------------------------------------------------------------------------


def _make_valid_nested_sm():
    """A valid SM: outcome1 loops, outcome2 -> done."""
    sm = StateMachine(outcomes=["done"])
    sm.add_state("WORK", BarState(), {"outcome1": "WORK", "outcome2": "done"})
    return sm


def _make_strict_invalid_nested_sm():
    """A strict-invalid SM: BarState outcome2 has no transition."""
    sm = StateMachine(outcomes=["done"])
    sm.add_state("WORK", BarState(), {"outcome1": "done"})
    return sm


class TestConcurrenceValidate(unittest.TestCase):
    def test_validate_passes_for_valid_nested_state_machine(self):
        conc = Concurrence(
            states={"NESTED": _make_valid_nested_sm()},
            default_outcome="done",
            outcome_map={"done": {"NESTED": "done"}},
        )
        # Should not raise
        conc.validate()

    def test_validate_throws_when_nested_state_machine_is_strict_invalid(self):
        conc = Concurrence(
            states={"NESTED": _make_strict_invalid_nested_sm()},
            default_outcome="done",
            outcome_map={"done": {"NESTED": "done"}},
        )
        with self.assertRaises(RuntimeError):
            conc.validate(True)

    def test_validate_passes_when_nested_sm_only_strict_invalid(self):
        # Non-strict mode must not raise even if strict would.
        conc = Concurrence(
            states={"NESTED": _make_strict_invalid_nested_sm()},
            default_outcome="done",
            outcome_map={"done": {"NESTED": "done"}},
        )
        conc.validate(False)


if __name__ == "__main__":
    unittest.main()
