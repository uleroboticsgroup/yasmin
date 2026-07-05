# Copyright (C) 2025 Miguel Ángel González Santamarta
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

import time
import unittest

from yasmin import Blackboard
from yasmin import State
from yasmin import StateMachine
from yasmin import JoinState
from yasmin import OrthogonalState


class StateA(State):
    def __init__(self):
        super().__init__(["done"])

    def execute(self, blackboard):
        time.sleep(0.05)
        return "done"


class StateB(State):
    def __init__(self):
        super().__init__(["done"])

    def execute(self, blackboard):
        time.sleep(0.1)
        return "done"


class TestOrthogonalState(unittest.TestCase):
    def setUp(self):
        self.blackboard = Blackboard()

    def _make_region(self, name, state):
        sm = StateMachine(["done"])
        sm.set_name(name)
        sm.add_state("work", state, {"done": "done"})
        sm.set_start_state("work")
        return sm

    def _make_synced_region(self, name, sync_id, post_sync_state):
        sm = StateMachine(["done"])
        sm.set_name(name)
        sm.add_state("work", StateA(), {"done": "sync"})
        sm.add_state("sync", JoinState(sync_id), {"joined": "finish"})
        sm.add_state("finish", post_sync_state, {"done": "done"})
        sm.set_start_state("work")
        return sm

    def test_basic_concurrent_regions(self):
        ort = OrthogonalState("timeout")
        ort.add_region("A", self._make_region("A", StateA()))
        ort.add_region("B", self._make_region("B", StateB()))
        result = ort(self.blackboard)
        self.assertEqual(result, "timeout")

    def test_outcome_map(self):
        ort = OrthogonalState("timeout", {"success": {"A": "done", "B": "done"}})
        ort.add_region("A", self._make_region("A", StateA()))
        ort.add_region("B", self._make_region("B", StateB()))
        result = ort(self.blackboard)
        self.assertEqual(result, "success")

    def test_sync_barrier(self):
        ort = OrthogonalState("timeout", {"success": {"A": "done", "B": "done"}})
        post_sync = StateA()
        ort.add_region("A", self._make_synced_region("A", "sync1", post_sync))
        ort.add_region("B", self._make_synced_region("B", "sync1", post_sync))
        result = ort(self.blackboard)
        self.assertEqual(result, "success")

    def test_cancel(self):
        ort = OrthogonalState("timeout", {"success": {"A": "done", "B": "done"}})
        ort.add_region("A", self._make_region("A", StateA()))
        ort.add_region("B", self._make_region("B", StateB()))
        self.assertFalse(ort.is_canceled())
        ort.cancel_state()
        self.assertTrue(ort.is_canceled())

    def test_configure_runs_once(self):
        ort = OrthogonalState("timeout", {"success": {"A": "done", "B": "done"}})
        ort.add_region("A", self._make_region("A", StateA()))
        ort.add_region("B", self._make_region("B", StateB()))
        ort.configure()
        ort.configure()
        result = ort(self.blackboard)
        self.assertEqual(result, "success")

    def test_str(self):
        ort = OrthogonalState("timeout")
        ort.add_region("A", self._make_region("A", StateA()))
        ort.add_region("B", self._make_region("B", StateB()))
        s = str(ort)
        self.assertIn("A", s)
        self.assertIn("B", s)

    def test_join_state_without_barrier(self):
        js = JoinState("sync_1")
        bb = Blackboard()
        result = js(bb)
        self.assertEqual(result, "joined")

    def test_join_state_custom_outcome(self):
        js = JoinState("sync_1", "all_arrived")
        bb = Blackboard()
        result = js(bb)
        self.assertEqual(result, "all_arrived")


# ---------------------------------------------------------------------------
# Validate: OrthogonalState recursively validates region StateMachines
# ---------------------------------------------------------------------------


class _StateWithExtraOutcome(State):
    """State with two outcomes so strict mode can detect a missing transition."""

    def __init__(self):
        super().__init__(["done", "extra"])

    def execute(self, blackboard):
        return "done"


def _make_valid_region_sm():
    sm = StateMachine(["done"])
    sm.add_state("WORK", StateA(), {"done": "done"})
    return sm


def _make_strict_invalid_region_sm():
    """Strict-invalid: "extra" outcome of WORK has no transition."""
    sm = StateMachine(["done"])
    sm.add_state("WORK", _StateWithExtraOutcome(), {"done": "done"})
    return sm


class TestOrthogonalStateValidate(unittest.TestCase):
    def test_validate_passes_for_valid_regions(self):
        ort = OrthogonalState("timeout", {"done": {"A": "done", "B": "done"}})
        ort.add_region("A", _make_valid_region_sm())
        ort.add_region("B", _make_valid_region_sm())
        # Should not raise
        ort.validate()

    def test_validate_throws_when_region_is_strict_invalid(self):
        ort = OrthogonalState("timeout", {"done": {"A": "done"}})
        ort.add_region("A", _make_strict_invalid_region_sm())
        with self.assertRaises(RuntimeError):
            ort.validate(True)

    def test_validate_passes_when_region_only_strict_invalid(self):
        # Non-strict mode must not raise even if strict would.
        ort = OrthogonalState("timeout", {"done": {"A": "done"}})
        ort.add_region("A", _make_strict_invalid_region_sm())
        ort.validate(False)


if __name__ == "__main__":
    unittest.main()
