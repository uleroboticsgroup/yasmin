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

from typing import Dict, Optional

ShellContextPayload = Dict[str, object]


def runtime_shell_allowed(runtime) -> bool:
    """Return whether the runtime exposes enough state for the shell."""

    return bool(
        runtime is not None
        and runtime.is_ready()
        and (
            getattr(runtime, "shell_bb", None) is not None
            or getattr(runtime, "bb", None) is not None
        )
        and getattr(runtime, "sm", None) is not None
    )


def runtime_shell_command_result(runtime, command_name: str) -> str:
    """Return the short status line shown after one shell command."""

    normalized_name = str(command_name).strip() or "command"
    if runtime is None:
        return f"{normalized_name}: runtime unavailable"

    current_state = getattr(runtime, "get_current_state", lambda: None)()
    last_state_ref = getattr(runtime, "get_last_state_ref", lambda: None)()
    last_state_name = getattr(last_state_ref, "name", None)
    if last_state_name is None and last_state_ref is not None:
        last_state_name = str(last_state_ref)

    current_label = str(current_state) if current_state is not None else "None"
    last_label = str(last_state_name) if last_state_name is not None else "None"
    return (
        f"{normalized_name}: status={runtime.get_status_label()}, "
        f"current_state={current_label}, last_state={last_label}"
    )


def runtime_shell_where_text(runtime) -> str:
    """Return the multi-line shell summary of the current runtime location."""

    if runtime is None:
        return "runtime unavailable"

    active_path = " / ".join(runtime.get_active_path() or tuple()) or "<none>"
    last_transition = runtime.get_last_transition()
    if last_transition is None:
        transition_label = "<none>"
    else:
        from_path, to_path, outcome = last_transition
        transition_label = (
            f"{' / '.join(from_path)} --[{outcome}]--> {' / '.join(to_path)}"
        )

    return (
        f"status={runtime.get_status_label()}\n"
        f"active_path={active_path}\n"
        f"last_transition={transition_label}"
    )


def build_runtime_shell_context_payload(
    runtime,
    commands: dict[str, object],
) -> Optional[ShellContextPayload]:
    """Return the shell namespace payload or ``None`` when unavailable."""

    if not runtime_shell_allowed(runtime):
        return None

    return {
        "bb": getattr(runtime, "shell_bb", None) or runtime.bb,
        "sm": runtime.sm,
        "current_state": runtime.get_current_state_ref(),
        "last_state": runtime.get_last_state_ref(),
        "commands": commands,
        "active_path": runtime.get_active_path(),
        "last_transition": runtime.get_last_transition(),
    }
