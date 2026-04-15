# Copyright (C) 2026 Maik Knof
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Tests for runtime helper modules that stay independent from Qt.

The covered helpers build runtime view state, translate transitions and paths,
manage breakpoints, and format the context exposed to the interactive shell.
"""

from types import SimpleNamespace

from yasmin_editor.editor_gui.runtime_breakpoints import (
    breakpoint_parent_path,
    breakpoint_tooltip,
    state_breakpoint_path,
    toggle_breakpoint_before,
)
from yasmin_editor.editor_gui.runtime_shell_context import (
    build_runtime_shell_context_payload,
    runtime_shell_allowed,
    runtime_shell_command_result,
    runtime_shell_where_text,
)
from yasmin_editor.editor_gui.runtime_state import (
    build_runtime_view_state,
    current_runtime_container_path,
    local_runtime_transition,
    normalize_runtime_path,
    root_final_transition,
    runtime_button_states,
    runtime_state_name_for_container,
)
from yasmin_editor.model.transition import Transition


class FakeRuntime:
    """Runtime stub that exposes the methods used by the pure helpers."""

    def __init__(
        self,
        *,
        ready=True,
        running=True,
        blocked=False,
        step_mode=False,
        finished=False,
        active_path=("root", "worker"),
        status="running",
        current_state="worker",
        current_state_ref=None,
        last_state_ref=None,
        last_transition=(("root",), ("done",), "success"),
        bb=None,
        sm=None,
    ) -> None:
        self._ready = ready
        self._running = running
        self._blocked = blocked
        self._step_mode = step_mode
        self._finished = finished
        self._active_path = tuple(active_path)
        self._status = status
        self._current_state = current_state
        self._current_state_ref = (
            current_state_ref
            if current_state_ref is not None
            else SimpleNamespace(name="worker")
        )
        self._last_state_ref = (
            last_state_ref
            if last_state_ref is not None
            else SimpleNamespace(name="last_worker")
        )
        self._last_transition = last_transition
        self.bb = {} if bb is None else bb
        self.sm = object() if sm is None else sm

    def is_ready(self):
        return self._ready

    def is_running(self):
        return self._running

    def is_blocked(self):
        return self._blocked

    def is_step_mode(self):
        return self._step_mode

    def is_finished(self):
        return self._finished

    def get_current_state(self):
        return self._current_state

    def get_current_state_ref(self):
        return self._current_state_ref

    def get_last_state_ref(self):
        return self._last_state_ref

    def get_status_label(self):
        return self._status

    def get_active_path(self):
        return self._active_path

    def get_last_transition(self):
        return self._last_transition


def test_runtime_state_helpers_compute_paths_visibility_and_local_transitions():
    """Runtime path helpers should project global runtime state into one container view."""

    current_container_path = [
        SimpleNamespace(name="root"),
        SimpleNamespace(name="nested"),
    ]
    assert normalize_runtime_path(["", "root", None, "worker"]) == (
        "root",
        "None",
        "worker",
    )
    assert current_runtime_container_path(
        ("runtime_nested",), current_container_path
    ) == ("runtime_nested",)
    assert current_runtime_container_path(None, current_container_path) == ("nested",)
    assert current_runtime_container_path(None, []) == ()

    runtime_state = build_runtime_view_state(
        True,
        FakeRuntime(
            ready=True, running=True, blocked=False, step_mode=False, finished=False
        ),
        shell_execution_blocked=True,
    )
    buttons = runtime_button_states(runtime_state)
    assert buttons["runtime_pause_button"].visible is True
    assert buttons["runtime_play_button"].enabled is False
    assert buttons["runtime_shell_button"].enabled is True

    blocked_state = build_runtime_view_state(
        True,
        FakeRuntime(
            ready=True, running=True, blocked=True, step_mode=True, finished=False
        ),
    )
    blocked_buttons = runtime_button_states(blocked_state)
    assert blocked_buttons["runtime_play_button"].visible is True
    assert blocked_buttons["runtime_pause_button"].visible is False
    assert blocked_buttons["runtime_step_button"].visible is True

    assert runtime_state_name_for_container(("nested", "worker"), ("nested",)) == "worker"
    assert runtime_state_name_for_container(("nested",), ("nested",)) is None
    assert runtime_state_name_for_container(("other", "worker"), ("nested",)) is None

    local = local_runtime_transition(
        (("nested", "worker"), ("nested", "done"), "success"),
        ("nested",),
    )
    assert local is not None
    assert (local.from_name, local.to_name, local.outcome) == (
        "worker",
        "done",
        "success",
    )
    assert (
        local_runtime_transition(
            (("nested", "worker", "inner"), ("nested", "done"), "success"),
            ("nested",),
        )
        is None
    )


def test_root_final_transition_finds_matching_final_outcome_target():
    """Finished root transitions should resolve only when the target outcome matches."""

    transitions = {
        "root_state": [
            Transition(source_outcome="done", target="finished"),
            Transition(source_outcome="failed", target="aborted"),
        ]
    }

    assert root_final_transition("root_state", transitions, "finished") == (
        ("root_state",),
        ("finished",),
        "done",
    )
    assert root_final_transition("root_state", transitions, "missing") is None
    assert root_final_transition("", transitions, "finished") is None


def test_runtime_breakpoint_helpers_toggle_paths_and_tooltips():
    """Breakpoint helpers should normalize state paths and report add or remove actions."""

    assert breakpoint_parent_path(False, ("nested",)) == ()
    assert breakpoint_parent_path(True, ["nested"]) == ("nested",)
    assert state_breakpoint_path(True, ("nested",), " worker ") == ("nested", "worker")
    assert state_breakpoint_path(True, ("nested",), "   ") == ()

    updated, action = toggle_breakpoint_before(
        {("nested", "worker")}, ("nested", "worker")
    )
    assert updated == set()
    assert action == "removed"
    updated, action = toggle_breakpoint_before(set(), ["nested", "worker"])
    assert updated == {("nested", "worker")}
    assert action == "added"
    assert breakpoint_tooltip(("nested", "worker"), updated) == "Breakpoint: break before"
    assert breakpoint_tooltip(("nested", "other"), updated) == ""


def test_runtime_shell_helpers_build_status_text_and_context_payload():
    """Shell helpers should format runtime location and expose the shell namespace."""

    runtime = FakeRuntime(
        ready=True,
        running=True,
        blocked=False,
        step_mode=False,
        finished=False,
        active_path=("root", "worker"),
        status="blocked",
        current_state="worker",
        current_state_ref=SimpleNamespace(name="worker"),
        last_state_ref=SimpleNamespace(name="last_worker"),
        last_transition=(("root", "worker"), ("root", "done"), "success"),
        bb={"count": 1},
        sm="state_machine_ref",
    )

    assert runtime_shell_allowed(runtime) is True
    missing_bb_runtime = FakeRuntime(sm="state_machine_ref")
    missing_bb_runtime.bb = None
    missing_sm_runtime = FakeRuntime(bb={"count": 1})
    missing_sm_runtime.sm = None
    assert runtime_shell_allowed(missing_bb_runtime) is False
    assert runtime_shell_allowed(missing_sm_runtime) is False

    result_text = runtime_shell_command_result(runtime, " inspect ")
    assert result_text == (
        "inspect: status=blocked, current_state=worker, last_state=last_worker"
    )
    assert runtime_shell_command_result(None, "inspect") == "inspect: runtime unavailable"

    where_text = runtime_shell_where_text(runtime)
    assert where_text == (
        "status=blocked\n"
        "active_path=root / worker\n"
        "last_transition=root / worker --[success]--> root / done"
    )
    assert runtime_shell_where_text(None) == "runtime unavailable"

    commands = {"where": object()}
    payload = build_runtime_shell_context_payload(runtime, commands)
    assert payload == {
        "bb": {"count": 1},
        "sm": "state_machine_ref",
        "current_state": runtime.get_current_state_ref(),
        "last_state": runtime.get_last_state_ref(),
        "commands": commands,
        "active_path": ("root", "worker"),
        "last_transition": (("root", "worker"), ("root", "done"), "success"),
    }
    missing_payload_runtime = FakeRuntime(sm="state_machine_ref")
    missing_payload_runtime.bb = None
    assert build_runtime_shell_context_payload(missing_payload_runtime, {}) is None
