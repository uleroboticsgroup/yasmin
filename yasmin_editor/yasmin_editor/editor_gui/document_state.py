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
"""Helpers for document title and dirty-state tracking."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from yasmin_editor.model.state_machine import StateMachine

WINDOW_TITLE = "YASMIN Editor"
UNTITLED_DOCUMENT_NAME = "Untitled"


class EditorDirtyTracker:
    """Track whether the current root model differs from the saved baseline."""

    def __init__(self) -> None:
        self._saved_root_model: StateMachine | None = None

    def reset(self, root_model: StateMachine) -> None:
        """Reset the saved baseline to the provided root model."""

        self._saved_root_model = deepcopy(root_model)

    def is_dirty(self, root_model: StateMachine) -> bool:
        """Return whether the provided root model differs from the baseline."""

        if self._saved_root_model is None:
            return False
        return root_model != self._saved_root_model


def document_display_name(file_path: str | None) -> str:
    """Return the user-visible document name for the current file path."""

    if not file_path:
        return UNTITLED_DOCUMENT_NAME
    return Path(file_path).name


def build_window_title(file_path: str | None, *, is_dirty: bool) -> str:
    """Return the main-window title for the current document state."""

    prefix = "* " if is_dirty else ""
    return f"{prefix}{document_display_name(file_path)} - {WINDOW_TITLE}"
