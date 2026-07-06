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

from copy import deepcopy
from pathlib import Path

from typing import Union

from yasmin_editor.model.state_machine import StateMachine

WINDOW_TITLE = "YASMIN Editor"
UNTITLED_DOCUMENT_NAME = "Untitled"


class EditorDirtyTracker:
    """Track whether the current root model differs from the saved baseline."""

    def __init__(self) -> None:
        self._saved_root_model: Union[StateMachine, None] = None

    def reset(self, root_model: StateMachine) -> None:
        """Reset the saved baseline to the provided root model."""

        self._saved_root_model = deepcopy(root_model)

    def is_dirty(self, root_model: StateMachine) -> bool:
        """Return whether the provided root model differs from the baseline."""

        if self._saved_root_model is None:
            return False
        return root_model != self._saved_root_model


def document_display_name(file_path: Union[str, None]) -> str:
    """Return the user-visible document name for the current file path."""

    if not file_path:
        return UNTITLED_DOCUMENT_NAME
    return Path(file_path).name


def build_window_title(file_path: Union[str, None], *, is_dirty: bool) -> str:
    """Return the main-window title for the current document state."""

    prefix = "* " if is_dirty else ""
    return f"{prefix}{document_display_name(file_path)} - {WINDOW_TITLE}"
