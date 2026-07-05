# Copyright (C) 2026 Maik Knof
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

import pytest

from yasmin_editor.editor_gui.final_outcome_ops import ensure_final_outcome_alias
from yasmin_editor.editor_gui.transition_rules import (
    TransitionRuleError,
    get_available_transition_outcomes,
    validate_drag_target,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.runtime.traversal import (
    child_state,
    container_states,
    expand_to_deepest_known_path,
    get_container_entry_state_name,
    resolve_container,
)


class PyContainer:
    def __init__(self, states=None, start_state=None, current_state=None):
        self._states = states or {}
        self.start_state = start_state
        self._current_state = current_state

    def get_states(self):
        return self._states

    def get_start_state(self):
        return self.start_state

    def get_current_state(self):
        return self._current_state


class MappingLike:
    def __init__(self, state):
        self._state = state

    def get(self, key, default=None):
        if key == "state":
            return self._state
        return default


class CppContainer:
    def __init__(self, states):
        self._states = states

    def _get_states_cpp(self):
        return self._states


class BrokenContainer:
    def get_states(self):
        raise RuntimeError("boom")


class ConcurrenceLike(PyContainer):
    pass


ConcurrenceLike.__name__ = "Concurrence"


def test_ensure_final_outcome_alias_creates_outcome_and_sets_concurrence_default():
    container = Concurrence(name="cc")

    result = ensure_final_outcome_alias(container, "done", 1.0, 2.0)

    assert result.created_outcome is True
    assert result.outcome.name == "done"
    assert result.instance_id
    assert container.default_outcome == "done"
    assert container.layout.get_outcome_position("done").x == 1.0


def test_ensure_final_outcome_alias_reuses_existing_outcome():
    container = StateMachine(name="sm")
    container.add_outcome(Outcome("done"))

    result = ensure_final_outcome_alias(container, "done", 3.0, 4.0)

    assert result.created_outcome is False
    assert len(container.outcomes) == 1
    assert container.layout.get_outcome_position("done").y == 4.0


def test_validate_drag_target_and_available_transition_outcomes():
    with pytest.raises(TransitionRuleError, match="without outcomes"):
        get_available_transition_outcomes(StateMachine(name="sm"), [])

    sm = StateMachine(name="sm")
    assert get_available_transition_outcomes(sm, ["ok", "fail"], {"ok"}) == ["fail"]

    with pytest.raises(TransitionRuleError, match="already used"):
        get_available_transition_outcomes(sm, ["ok"], {"ok"})

    cc = Concurrence(name="cc")
    validate_drag_target(cc, from_is_final_outcome=False, to_is_final_outcome=True)

    with pytest.raises(TransitionRuleError, match="can only connect"):
        validate_drag_target(cc, from_is_final_outcome=False, to_is_final_outcome=False)

    with pytest.raises(TransitionRuleError, match="cannot start transitions"):
        validate_drag_target(sm, from_is_final_outcome=True, to_is_final_outcome=False)


def test_container_states_normalizes_python_and_cpp_containers():
    leaf = object()
    py_container = PyContainer(
        {
            "a": {"state": leaf},
            "b": MappingLike("wrapped"),
        }
    )
    cpp_container = CppContainer({"c": leaf})

    assert container_states(py_container) == {"a": leaf, "b": "wrapped"}
    assert container_states(cpp_container) == {"c": leaf}
    assert container_states(BrokenContainer()) == {}
    assert child_state(py_container, "a") is leaf


def test_runtime_traversal_resolves_nested_paths_and_stops_at_concurrence_like_objects():
    leaf = object()
    nested = PyContainer({"leaf": leaf}, start_state="leaf")
    root = PyContainer({"nested": nested}, start_state="nested")

    assert resolve_container(root, ("nested",)) is nested
    assert get_container_entry_state_name(root) == "nested"
    assert expand_to_deepest_known_path(root, ()) == ("nested", "leaf")

    concurrence_like = ConcurrenceLike({"leaf": leaf}, start_state="leaf")
    root_with_cc = PyContainer({"parallel": concurrence_like}, start_state="parallel")
    assert expand_to_deepest_known_path(root_with_cc, ()) == ("parallel",)
