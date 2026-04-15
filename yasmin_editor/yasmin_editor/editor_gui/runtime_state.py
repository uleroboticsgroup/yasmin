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

"""Pure runtime state helpers shared by the editor runtime mixin.

The runtime mixin still owns the Qt-facing orchestration, but several decisions
inside it are not inherently graphical: button visibility, the currently active
state inside one container view, and the local transition that should be
highlighted. Keeping those rules in one Qt-free module makes them easier to
review, document, and test without a live editor window.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True, slots=True)
class RuntimeViewState:
    """Snapshot of the runtime state needed by the editor controls."""

    mode_enabled: bool
    ready: bool
    running: bool
    blocked: bool
    playing: bool
    step_mode: bool
    finished: bool
    shell_execution_blocked: bool = False


@dataclass(frozen=True, slots=True)
class RuntimeButtonState:
    """Visibility and enabled state for one runtime control button."""

    visible: bool
    enabled: bool


@dataclass(frozen=True, slots=True)
class RuntimeLocalTransition:
    """Transition projected into the currently visible container."""

    from_name: str
    to_name: str
    outcome: str


def normalize_runtime_path(path: tuple[str, ...] | list[str] | None) -> tuple[str, ...]:
    """Normalize arbitrary runtime paths to a tuple of non-empty strings."""
    return tuple(str(item) for item in (path or tuple()) if str(item))


def current_runtime_container_path(
    explicit_path: tuple[str, ...] | list[str] | None,
    current_container_path,
) -> tuple[str, ...]:
    """Return the container path currently shown by the editor.

    Runtime mode can explicitly track the visible runtime container path. When
    that is not set yet, the editor falls back to the current model container
    stack, excluding the synthetic root entry.
    """

    normalized_explicit_path = normalize_runtime_path(explicit_path)
    if normalized_explicit_path:
        return normalized_explicit_path

    containers = tuple(current_container_path or tuple())
    if not containers:
        return tuple()

    return tuple(str(container.name) for container in containers[1:])


def build_runtime_view_state(
    runtime_mode_enabled: bool,
    runtime,
    shell_execution_blocked: bool = False,
) -> RuntimeViewState:
    """Compute the runtime-control flags used by the editor window."""

    ready = bool(runtime_mode_enabled and runtime is not None and runtime.is_ready())
    running = bool(ready and runtime.is_running())
    blocked = bool(running and runtime.is_blocked())
    playing = bool(running and not blocked)
    step_mode = bool(running and runtime.is_step_mode())
    finished = bool(ready and runtime.is_finished())

    return RuntimeViewState(
        mode_enabled=bool(runtime_mode_enabled),
        ready=ready,
        running=running,
        blocked=blocked,
        playing=playing,
        step_mode=step_mode,
        finished=finished,
        shell_execution_blocked=bool(shell_execution_blocked),
    )


def runtime_button_states(
    runtime_state: RuntimeViewState,
) -> dict[str, RuntimeButtonState]:
    """Return visibility/enabled rules for the runtime control buttons."""

    states = {
        "runtime_play_button": RuntimeButtonState(
            visible=runtime_state.ready
            and not runtime_state.playing
            and not runtime_state.finished,
            enabled=runtime_state.ready
            and not runtime_state.playing
            and not runtime_state.finished
            and not runtime_state.shell_execution_blocked,
        ),
        "runtime_pause_button": RuntimeButtonState(
            visible=runtime_state.playing and not runtime_state.step_mode,
            enabled=runtime_state.playing and not runtime_state.step_mode,
        ),
        "runtime_step_button": RuntimeButtonState(
            visible=runtime_state.ready
            and not runtime_state.playing
            and not runtime_state.finished,
            enabled=runtime_state.ready
            and not runtime_state.playing
            and not runtime_state.finished
            and not runtime_state.shell_execution_blocked,
        ),
        "runtime_cancel_state_button": RuntimeButtonState(
            visible=runtime_state.running and not runtime_state.finished,
            enabled=runtime_state.running and not runtime_state.finished,
        ),
        "runtime_cancel_sm_button": RuntimeButtonState(
            visible=runtime_state.running and not runtime_state.finished,
            enabled=runtime_state.running and not runtime_state.finished,
        ),
        "runtime_restart_button": RuntimeButtonState(
            visible=runtime_state.finished,
            enabled=runtime_state.finished,
        ),
        "runtime_auto_follow_button": RuntimeButtonState(
            visible=runtime_state.mode_enabled,
            enabled=runtime_state.ready,
        ),
        "runtime_shell_button": RuntimeButtonState(
            visible=runtime_state.mode_enabled,
            enabled=runtime_state.mode_enabled,
        ),
    }
    return states


def runtime_state_name_for_container(
    active_path: tuple[str, ...] | list[str] | None,
    current_path: tuple[str, ...] | list[str] | None,
) -> Optional[str]:
    """Return the active child state name visible in the current container."""

    normalized_active_path = normalize_runtime_path(active_path)
    normalized_current_path = normalize_runtime_path(current_path)

    if not normalized_active_path:
        return None
    if len(normalized_current_path) >= len(normalized_active_path):
        return None
    if normalized_active_path[: len(normalized_current_path)] != normalized_current_path:
        return None

    return normalized_active_path[len(normalized_current_path)]


def local_runtime_transition(
    transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    current_path: tuple[str, ...] | list[str] | None,
) -> Optional[RuntimeLocalTransition]:
    """Project one runtime transition into the currently visible container."""

    if transition is None:
        return None

    normalized_current_path = normalize_runtime_path(current_path)
    from_path, to_path, outcome = transition
    normalized_from_path = normalize_runtime_path(from_path)
    normalized_to_path = normalize_runtime_path(to_path)
    expected_depth = len(normalized_current_path) + 1

    if len(normalized_from_path) != expected_depth:
        return None
    if len(normalized_to_path) != expected_depth:
        return None
    if normalized_from_path[: len(normalized_current_path)] != normalized_current_path:
        return None
    if normalized_to_path[: len(normalized_current_path)] != normalized_current_path:
        return None

    return RuntimeLocalTransition(
        from_name=normalized_from_path[len(normalized_current_path)],
        to_name=normalized_to_path[len(normalized_current_path)],
        outcome=str(outcome),
    )


def root_final_transition(
    root_state_name: str,
    transitions_by_state: dict[str, list[object]],
    outcome: Optional[str],
) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
    """Resolve a finished root transition from the root container transitions."""

    normalized_outcome = str(outcome).strip() if outcome is not None else ""
    if not root_state_name or not normalized_outcome:
        return None

    for transition in transitions_by_state.get(root_state_name, []):
        if str(getattr(transition, "target", "")) == normalized_outcome:
            return (
                (str(root_state_name),),
                (normalized_outcome,),
                str(getattr(transition, "source_outcome", "")),
            )

    return None
