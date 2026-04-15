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
"""Editor undo/redo history helpers."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine

ContainerModel = StateMachine | Concurrence


@dataclass(slots=True)
class EditorHistorySnapshot:
    """Serializable editor state used by the history stack."""

    root_model: StateMachine
    container_path: tuple[str, ...] = tuple()
    current_file_path: str | None = None


class EditorHistory:
    """Small bounded undo/redo stack for editor snapshots."""

    def __init__(self, max_entries: int = 100) -> None:
        self.max_entries = max(2, int(max_entries))
        self._undo_stack: list[EditorHistorySnapshot] = []
        self._redo_stack: list[EditorHistorySnapshot] = []

    @property
    def can_undo(self) -> bool:
        """Return whether at least one earlier snapshot is available."""

        return len(self._undo_stack) >= 2

    @property
    def can_redo(self) -> bool:
        """Return whether one undone snapshot can be restored."""

        return bool(self._redo_stack)

    def _trim_undo_stack(self) -> None:
        if len(self._undo_stack) > self.max_entries:
            self._undo_stack = self._undo_stack[-self.max_entries :]

    def clear(self) -> None:
        """Remove all stored snapshots."""

        self._undo_stack.clear()
        self._redo_stack.clear()

    def reset(self, snapshot: EditorHistorySnapshot) -> None:
        """Replace the complete history with one baseline snapshot."""

        self._undo_stack = [deepcopy(snapshot)]
        self._redo_stack.clear()

    def record(self, snapshot: EditorHistorySnapshot) -> bool:
        """Append one snapshot when it differs from the latest entry."""

        stored_snapshot = deepcopy(snapshot)
        if self._undo_stack and self._undo_stack[-1] == stored_snapshot:
            return False

        self._undo_stack.append(stored_snapshot)
        self._trim_undo_stack()
        self._redo_stack.clear()
        return True

    def undo(
        self, current_snapshot: EditorHistorySnapshot
    ) -> EditorHistorySnapshot | None:
        """Return the previous snapshot if one is available."""

        current_state = deepcopy(current_snapshot)
        if not self._undo_stack:
            self._undo_stack.append(current_state)
            return None

        if self._undo_stack[-1] != current_state:
            self._undo_stack.append(current_state)
            self._trim_undo_stack()

        if len(self._undo_stack) < 2:
            return None

        current_entry = self._undo_stack.pop()
        self._redo_stack.append(deepcopy(current_entry))
        return deepcopy(self._undo_stack[-1])

    def redo(
        self, current_snapshot: EditorHistorySnapshot
    ) -> EditorHistorySnapshot | None:
        """Return the next snapshot if one is available."""

        if not self._redo_stack:
            return None

        current_state = deepcopy(current_snapshot)
        if self._undo_stack and self._undo_stack[-1] != current_state:
            self._redo_stack.clear()
            return None

        snapshot = deepcopy(self._redo_stack.pop())
        self._undo_stack.append(deepcopy(snapshot))
        self._trim_undo_stack()
        return snapshot


def container_path_names(container_path: list[ContainerModel]) -> tuple[str, ...]:
    """Return the nested container path relative to the root container."""

    return tuple(container.name for container in container_path[1:])


def resolve_container_path(
    root_model: StateMachine,
    path_names: tuple[str, ...],
) -> list[ContainerModel]:
    """Resolve one nested container path against the provided root model."""

    resolved_path: list[ContainerModel] = [root_model]
    current: ContainerModel = root_model

    for name in path_names:
        child = current.states.get(name)
        if child is None or not child.is_container:
            break
        resolved_path.append(child)
        current = child

    return resolved_path
