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

from typing import Iterable, Tuple, List, Set, Union
from yasmin_editor.editor_gui.runtime_state import normalize_runtime_path

BreakpointPath = Tuple[str, ...]


def breakpoint_parent_path(
    runtime_mode_enabled: bool,
    current_container_path: Union[Tuple[str, ...], List[str], None],
) -> BreakpointPath:
    """Return the runtime container path that owns visible breakpoint markers."""

    if not runtime_mode_enabled:
        return tuple()
    return normalize_runtime_path(current_container_path)


def state_breakpoint_path(
    runtime_mode_enabled: bool,
    current_container_path: Union[Tuple[str, ...], List[str], None],
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
) -> Tuple[Set[BreakpointPath], str]:
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
