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


import threading
import time
import unittest
from yasmin import Blackboard, StateMachine, State


class FooState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard):
        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"

        else:
            return "outcome2"


class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["outcome2", "outcome3"])

    def execute(self, blackboard):
        return "outcome2"


class TestStateMachine(unittest.TestCase):

    maxDiff = None

    def setUp(self):
        self.sm = StateMachine(outcomes=["outcome4", "outcome5"])

        self.sm.add_state(
            "FOO",
            FooState(),
            transitions={
                "outcome1": "BAR",
                "outcome2": "outcome4",
            },
        )
        self.sm.add_state(
            "BAR",
            BarState(),
            transitions={"outcome2": "FOO"},
        )

    def test_str(self):
        string = str(self.sm)
        # Check if "Bar (BarState)" and "Foo (FooState)" are in the string
        self.assertIn("BAR (BarState)", string)
        self.assertIn("FOO (FooState)", string)

    def test_get_name_empty(self):
        self.assertEqual("", self.sm.get_name())

    def test_set_name(self):
        self.sm.set_name("TestSM")
        self.assertEqual("TestSM", self.sm.get_name())

    def test_get_states(self):
        self.assertTrue(isinstance(self.sm.get_states()["FOO"]["state"], FooState))
        self.assertTrue(isinstance(self.sm.get_states()["BAR"]["state"], BarState))

    def test_get_start_state(self):
        self.assertEqual("FOO", self.sm.get_start_state())
        self.sm.set_start_state("BAR")
        self.assertEqual("BAR", self.sm.get_start_state())

    def test_get_current_state(self):
        self.assertEqual("", self.sm.get_current_state())

    def test_state_call(self):
        self.assertEqual("outcome4", self.sm())

    def test_set_start_state_empty(self):
        with self.assertRaises(ValueError) as context:
            self.sm.set_start_state("")
        self.assertEqual(str(context.exception), "Initial state cannot be empty")

    def test_set_start_state_wrong_state(self):
        with self.assertRaises(ValueError) as context:
            self.sm.set_start_state("FOO1")
        self.assertEqual(
            str(context.exception),
            "Initial state 'FOO1' is not in the state machine",
        )

    def test_add_repeated_state(self):
        with self.assertRaises(RuntimeError) as context:
            self.sm.add_state(
                "FOO",
                FooState(),
                transitions={
                    "outcome1": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "State 'FOO' already registered in the state machine",
        )

    def test_add_outcome_state(self):
        with self.assertRaises(RuntimeError) as context:
            self.sm.add_state(
                "outcome4",
                FooState(),
                transitions={
                    "outcome1": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "State name 'outcome4' is already registered as an outcome",
        )

    def test_add_state_with_wrong_outcome(self):
        with self.assertRaises(ValueError) as context:
            self.sm.add_state(
                "FOO1",
                FooState(),
                transitions={
                    "outcome9": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "State 'FOO1' references unregistered outcomes 'outcome9', available outcomes are ['outcome1', 'outcome2']",
        )

    def test_add_wrong_source_transition(self):
        with self.assertRaises(ValueError) as context:
            self.sm.add_state(
                "FOO1",
                FooState(),
                transitions={
                    "": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "Transitions with empty source in state 'FOO1'",
        )

    def test_add_wrong_target_transition(self):
        with self.assertRaises(ValueError) as context:
            self.sm.add_state(
                "FOO1",
                FooState(),
                transitions={
                    "outcome1": "",
                },
            )
        self.assertEqual(
            str(context.exception), "Transitions with empty target in state 'FOO1'"
        )

    def test_validate_outcome_from_fsm_not_used(self):

        sm_1 = StateMachine(outcomes=["outcome4"])

        sm_2 = StateMachine(outcomes=["outcome4", "outcome5"])
        sm_1.add_state("FSM", sm_2)

        sm_2.add_state(
            "FOO",
            FooState(),
            transitions={
                "outcome1": "outcome4",
                "outcome2": "outcome4",
            },
        )

        with self.assertRaises(RuntimeError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "State 'FSM' outcome 'outcome5' not registered in transitions",
        )

    def test_validate_outcome_from_state_not_used(self):

        sm_1 = StateMachine(outcomes=["outcome4"])

        sm_2 = StateMachine(outcomes=["outcome4"])
        sm_1.add_state("FSM", sm_2)

        sm_2.add_state(
            "FOO",
            FooState(),
            transitions={
                "outcome1": "outcome4",
            },
        )

        with self.assertRaises(RuntimeError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "State 'FOO' outcome 'outcome2' not registered in transitions",
        )

    def test_validate_fsm_outcome_not_used(self):

        sm_1 = StateMachine(outcomes=["outcome4"])

        sm_2 = StateMachine(outcomes=["outcome4", "outcome5"])
        sm_1.add_state(
            "FSM",
            sm_2,
            transitions={
                "outcome5": "outcome4",
            },
        )

        sm_2.add_state(
            "FOO",
            FooState(),
            transitions={
                "outcome1": "outcome4",
                "outcome2": "outcome4",
            },
        )

        with self.assertRaises(RuntimeError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "Target outcome 'outcome5' not registered in transitions",
        )

    def test_validate_wrong_state(self):

        sm_1 = StateMachine(outcomes=["outcome4"])

        sm_2 = StateMachine(outcomes=["outcome4"])
        sm_1.add_state(
            "FSM",
            sm_2,
            transitions={
                "outcome4": "outcome4",
            },
        )

        sm_2.add_state(
            "FOO",
            FooState(),
            transitions={
                "outcome1": "BAR",
                "outcome2": "outcome4",
            },
        )

        with self.assertRaises(RuntimeError) as context:
            sm_1.validate()
        self.assertEqual(
            str(context.exception),
            "State machine outcome 'BAR' not registered as outcome neither state",
        )

    def test_start_callback(self):
        start_called = False
        start_state_called = None

        def start_cb(blackboard, start_state):
            nonlocal start_called, start_state_called
            start_called = True
            start_state_called = start_state

        self.sm.add_start_cb(start_cb)
        result = self.sm()
        self.assertEqual(result, "outcome4")
        self.assertTrue(start_called)
        self.assertEqual(start_state_called, "FOO")

    def test_transition_callback(self):
        transitions_called = []

        def transition_cb(blackboard, from_state, to_state, outcome):
            transitions_called.append((from_state, to_state, outcome))

        self.sm.add_transition_cb(transition_cb)
        result = self.sm()
        self.assertEqual(result, "outcome4")

        # Should have transitions: FOO -> BAR (outcome1), BAR -> FOO (outcome2), FOO -> BAR (outcome1), BAR -> FOO (outcome2), FOO -> BAR (outcome1), BAR -> FOO (outcome2)
        self.assertEqual(len(transitions_called), 6)
        self.assertEqual(transitions_called[0], ("FOO", "BAR", "outcome1"))
        self.assertEqual(transitions_called[1], ("BAR", "FOO", "outcome2"))
        self.assertEqual(transitions_called[2], ("FOO", "BAR", "outcome1"))
        self.assertEqual(transitions_called[3], ("BAR", "FOO", "outcome2"))
        self.assertEqual(transitions_called[4], ("FOO", "BAR", "outcome1"))
        self.assertEqual(transitions_called[5], ("BAR", "FOO", "outcome2"))

    def test_end_callback(self):
        end_called = False
        end_outcome_called = None

        def end_cb(blackboard, outcome):
            nonlocal end_called, end_outcome_called
            end_called = True
            end_outcome_called = outcome

        self.sm.add_end_cb(end_cb)
        result = self.sm()
        self.assertEqual(result, "outcome4")
        self.assertTrue(end_called)
        self.assertEqual(end_outcome_called, "outcome4")

    def test_multiple_callbacks(self):
        start_count = [0]
        transition_count = [0]
        end_count = [0]

        self.sm.add_start_cb(
            lambda blackboard, start_state: start_count.__setitem__(0, start_count[0] + 1)
        )
        self.sm.add_start_cb(
            lambda blackboard, start_state: start_count.__setitem__(0, start_count[0] + 1)
        )

        self.sm.add_transition_cb(
            lambda blackboard, from_state, to_state, outcome: transition_count.__setitem__(
                0, transition_count[0] + 1
            )
        )
        self.sm.add_transition_cb(
            lambda blackboard, from_state, to_state, outcome: transition_count.__setitem__(
                0, transition_count[0] + 1
            )
        )

        self.sm.add_end_cb(
            lambda blackboard, outcome: end_count.__setitem__(0, end_count[0] + 1)
        )
        self.sm.add_end_cb(
            lambda blackboard, outcome: end_count.__setitem__(0, end_count[0] + 1)
        )

        result = self.sm()
        self.assertEqual(result, "outcome4")
        self.assertEqual(start_count[0], 2)
        self.assertEqual(transition_count[0], 12)
        self.assertEqual(end_count[0], 2)


class ConfigurableLeafState(State):
    def __init__(self):
        super().__init__(["done"])
        self.declare_parameter("topic", "Leaf topic")
        self.configure_count = 0
        self.configured_topic = None

    def configure(self):
        self.configure_count += 1
        self.configured_topic = self.get_parameter("topic")

    def execute(self, blackboard):
        return "done"


class TestStateMachineConfigure(unittest.TestCase):
    def test_configure_only_runs_once_on_repeated_execution(self):
        leaf = ConfigurableLeafState()
        sm = StateMachine(outcomes=["done"])
        sm.declare_parameter("topic", "Parent topic", "/demo")
        sm.add_state(
            "LEAF",
            leaf,
            transitions={"done": "done"},
            parameter_mappings={"topic": "topic"},
        )

        self.assertEqual("done", sm())
        self.assertEqual("done", sm())
        self.assertEqual(1, leaf.configure_count)
        self.assertEqual("/demo", leaf.configured_topic)

    def test_nested_configure_applies_parameter_mappings_recursively(self):
        leaf = ConfigurableLeafState()
        nested_sm = StateMachine(outcomes=["done"])
        nested_sm.declare_parameter("nested_topic", "Nested topic")
        nested_sm.add_state(
            "LEAF",
            leaf,
            transitions={"done": "done"},
            parameter_mappings={"topic": "nested_topic"},
        )

        root_sm = StateMachine(outcomes=["done"])
        root_sm.declare_parameter("root_topic", "Root topic", "/root")
        root_sm.add_state(
            "NESTED",
            nested_sm,
            transitions={"done": "done"},
            parameter_mappings={"nested_topic": "root_topic"},
        )

        self.assertEqual("done", root_sm())
        self.assertEqual(1, leaf.configure_count)
        self.assertEqual("/root", leaf.configured_topic)
        self.assertEqual("/root", nested_sm.get_parameter("nested_topic"))


class BlockingCancelState(State):
    def __init__(self, enter_event: threading.Event):
        super().__init__(outcomes=["done"])
        self.enter_event = enter_event

    def execute(self, blackboard):
        self.enter_event.set()
        while not self.is_canceled():
            time.sleep(0.01)
        return "done"


class TestStateMachineCancelBehavior(unittest.TestCase):

    def test_sequential_cancel_state_can_cancel_following_state(self):
        first_entered = threading.Event()
        second_entered = threading.Event()
        blackboard = Blackboard()
        result = {}

        sm = StateMachine(outcomes=["done"])
        sm.add_state(
            "FIRST",
            BlockingCancelState(first_entered),
            transitions={"done": "SECOND"},
        )
        sm.add_state(
            "SECOND",
            BlockingCancelState(second_entered),
            transitions={"done": "done"},
        )

        def run_state_machine():
            result["outcome"] = sm(blackboard)

        thread = threading.Thread(target=run_state_machine)
        thread.start()

        self.assertTrue(first_entered.wait(timeout=2.0))
        self.assertEqual(sm.get_current_state(), "FIRST")

        sm.cancel_state()

        self.assertTrue(second_entered.wait(timeout=2.0))
        self.assertEqual(sm.get_current_state(), "SECOND")

        sm.cancel_state()
        thread.join(timeout=2.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result.get("outcome"), "done")

    def test_cancel_state_machine_stops_execution(self):
        first_entered = threading.Event()
        second_entered = threading.Event()
        blackboard = Blackboard()
        result = {}

        sm = StateMachine(outcomes=["done"])
        sm.add_state(
            "FIRST",
            BlockingCancelState(first_entered),
            transitions={"done": "SECOND"},
        )
        sm.add_state(
            "SECOND",
            BlockingCancelState(second_entered),
            transitions={"done": "done"},
        )

        def run_state_machine():
            try:
                sm(blackboard)
            except Exception as exc:
                result["exception"] = exc

        thread = threading.Thread(target=run_state_machine)
        thread.start()

        self.assertTrue(first_entered.wait(timeout=2.0))
        self.assertEqual(sm.get_current_state(), "FIRST")

        sm.cancel_state_machine()
        thread.join(timeout=2.0)

        self.assertFalse(thread.is_alive())
        self.assertIn("State machine canceled:", str(result.get("exception")))
        self.assertFalse(second_entered.is_set())
        self.assertEqual(sm.get_current_state(), "")


if __name__ == "__main__":
    unittest.main()
