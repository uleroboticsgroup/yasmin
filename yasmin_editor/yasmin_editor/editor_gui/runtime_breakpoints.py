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

"""Pure runtime-breakpoint helpers used by the editor runtime mixin.

The editor runtime mixin still owns the Qt menu and status-bar wiring, but the
breakpoint path rules themselves are independent of Qt. Keeping them here makes
it easier to validate breakpoint behavior without constructing editor widgets.
"""

from __future__ import annotations

from typing import Iterable

from yasmin_editor.editor_gui.runtime_state import normalize_runtime_path

BreakpointPath = tuple[str, ...]


def breakpoint_parent_path(
    runtime_mode_enabled: bool,
    current_container_path: tuple[str, ...] | list[str] | None,
) -> BreakpointPath:
    """Return the runtime container path that owns visible breakpoint markers."""

    if not runtime_mode_enabled:
        return tuple()
    return normalize_runtime_path(current_container_path)


def state_breakpoint_path(
    runtime_mode_enabled: bool,
    current_container_path: tuple[str, ...] | list[str] | None,
    state_name: object,
) -> BreakpointPath:
    """Return the full runtime breakpoint path for one visible state node."""

    normalized_state_name = str(state_name).strip()
    if not normalized_state_name:
        return tuple()

    return normalize_runtime_path(
        breakpoint_parent_path(runtime_mode_enabled, current_container_path)
        + (normalized_state_name,)
    )


def breakpoint_tooltip(
    state_path: BreakpointPath,
    before_paths: Iterable[BreakpointPath],
) -> str:
    """Return the tooltip text for one breakpoint marker."""

    return "Breakpoint: break before" if state_path in set(before_paths) else ""


def toggle_breakpoint_before(
    before_paths: Iterable[BreakpointPath],
    state_path: BreakpointPath,
) -> tuple[set[BreakpointPath], str]:
    """Toggle one breakpoint path and return the updated set and action label."""

    updated_paths = set(before_paths)
    normalized_state_path = normalize_runtime_path(state_path)
    if not normalized_state_path:
        return updated_paths, ""

    if normalized_state_path in updated_paths:
        updated_paths.remove(normalized_state_path)
        return updated_paths, "removed"

    updated_paths.add(normalized_state_path)
    return updated_paths, "added"
