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
from yasmin import StateMachine, State


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
        self.assertEqual("State Machine [BAR (BarState), FOO (FooState)]", str(self.sm))

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
        with self.assertRaises(KeyError) as context:
            self.sm.set_start_state("FOO1")
        self.assertEqual(
            str(context.exception),
            "\"Initial state 'FOO1' is not in the state machine\"",
        )

    def test_add_repeated_state(self):
        with self.assertRaises(KeyError) as context:
            self.sm.add_state(
                "FOO",
                FooState(),
                transitions={
                    "outcome1": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "\"State 'FOO' already registered in the state machine\"",
        )

    def test_add_state_with_wrong_outcome(self):
        with self.assertRaises(KeyError) as context:
            self.sm.add_state(
                "FOO1",
                FooState(),
                transitions={
                    "outcome9": "BAR",
                },
            )
        self.assertEqual(
            str(context.exception),
            "\"State 'FOO1' references unregistered outcomes 'outcome9', available outcomes are ['outcome1', 'outcome2']\"",
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

        with self.assertRaises(KeyError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "\"State 'FSM' outcome 'outcome5' not registered in transitions\"",
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

        with self.assertRaises(KeyError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "\"State 'FOO' outcome 'outcome2' not registered in transitions\"",
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

        with self.assertRaises(KeyError) as context:
            sm_1.validate(True)
        self.assertEqual(
            str(context.exception),
            "\"Target outcome 'outcome5' not registered in transitions\"",
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

        with self.assertRaises(KeyError) as context:
            sm_1.validate()
        self.assertEqual(
            str(context.exception),
            "\"State machine outcome 'BAR' not registered as outcome neither state\"",
        )
