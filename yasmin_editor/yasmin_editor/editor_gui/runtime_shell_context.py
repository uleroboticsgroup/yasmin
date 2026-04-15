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

"""Pure helpers for the runtime interactive shell integration.

The interactive shell is optional UI, but its context payload and status text
are plain data derivations from the runtime backend. Keeping those rules here
lets the test suite cover shell behavior without importing Qt widgets.
"""

from __future__ import annotations

from typing import Optional

ShellContextPayload = dict[str, object]


def runtime_shell_allowed(runtime) -> bool:
    """Return whether the runtime exposes enough state for the shell."""

    return bool(
        runtime is not None
        and runtime.is_ready()
        and getattr(runtime, "bb", None) is not None
        and getattr(runtime, "sm", None) is not None
    )


def runtime_shell_command_result(runtime, command_name: str) -> str:
    """Return the short status line shown after one shell command."""

    normalized_name = str(command_name).strip() or "command"
    if runtime is None:
        return f"{normalized_name}: runtime unavailable"

    current_state = runtime.get_current_state()
    last_state_ref = runtime.get_last_state_ref()
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
        "bb": runtime.bb,
        "sm": runtime.sm,
        "current_state": runtime.get_current_state_ref(),
        "last_state": runtime.get_last_state_ref(),
        "commands": commands,
        "active_path": runtime.get_active_path(),
        "last_transition": runtime.get_last_transition(),
    }
