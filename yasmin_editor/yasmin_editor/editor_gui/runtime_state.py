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

from __future__ import annotations

from yasmin_editor.dataclass_compat import dataclass
from typing import Optional, Tuple, List, Dict, Union


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


def normalize_runtime_path(
    path: Union[Tuple[str, ...], List[str], None],
) -> Tuple[str, ...]:
    """Normalize arbitrary runtime paths to a tuple of non-empty strings."""
    return tuple(str(item) for item in (path or tuple()) if str(item))


def current_runtime_container_path(
    explicit_path: Union[Tuple[str, ...], List[str], None],
    current_container_path: Union[Tuple[str, ...], List[str], None],
) -> Tuple[str, ...]:
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
    active_path: Union[Tuple[str, ...], List[str], None],
    current_path: Union[Tuple[str, ...], List[str], None],
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
    transition: Optional[Tuple[Tuple[str, ...], Tuple[str, ...], str]],
    current_path: Union[Tuple[str, ...], List[str], None],
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
    transitions_by_state: Dict[str, List[object]],
    outcome: Optional[str],
) -> Optional[Tuple[Tuple[str, ...], Tuple[str, ...], str]]:
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
