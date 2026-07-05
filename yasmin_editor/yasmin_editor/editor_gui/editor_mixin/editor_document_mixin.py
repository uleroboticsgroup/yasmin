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

from yasmin_editor.qt_compat import QtWidgets
from yasmin_editor.editor_gui.document_state import (
    EditorDirtyTracker,
    build_window_title,
    document_display_name,
)
from yasmin_editor.editor_gui.recent_files import RecentFilesStore


class EditorDocumentMixin:
    """Mixin for tracking unsaved document changes and window title state."""

    def _create_document_state(self) -> None:
        self.dirty_tracker = EditorDirtyTracker()
        self.recent_files_store = RecentFilesStore()
        self.recent_files = self.recent_files_store.load_entries(existing_only=True)

    def register_recent_file(self, file_path: str) -> None:
        """Persist one document path in the recent-files list."""

        if not file_path:
            return
        self.recent_files = self.recent_files_store.add_file(
            file_path,
            current_entries=self.recent_files,
        )
        if hasattr(self, "recent_files_menu"):
            from yasmin_editor.editor_gui.ui.recent_menu import refresh_recent_file_menu

            refresh_recent_file_menu(self)

    def remove_recent_file(self, file_path: str) -> None:
        """Remove one document path from the recent-files list."""

        if not file_path:
            return
        self.recent_files = self.recent_files_store.remove_file(
            file_path,
            current_entries=self.recent_files,
        )
        if hasattr(self, "recent_files_menu"):
            from yasmin_editor.editor_gui.ui.recent_menu import refresh_recent_file_menu

            refresh_recent_file_menu(self)

    def clear_recent_files(self) -> None:
        """Clear the persisted recent-files list."""

        self.recent_files = []
        self.recent_files_store.clear()
        if hasattr(self, "recent_files_menu"):
            from yasmin_editor.editor_gui.ui.recent_menu import refresh_recent_file_menu

            refresh_recent_file_menu(self)
        self.statusBar().showMessage("Recent files cleared", 2000)

    def is_document_dirty(self) -> bool:
        """Return whether the editor contains unsaved changes."""

        if not hasattr(self, "dirty_tracker"):
            return False
        return self.dirty_tracker.is_dirty(self.root_model)

    def reset_document_dirty_state(self) -> None:
        """Mark the current root model as the new clean baseline."""

        if not hasattr(self, "dirty_tracker"):
            return
        self.dirty_tracker.reset(self.root_model)
        self.update_document_window_title()

    def update_document_window_title(self) -> None:
        """Refresh the window title from file path and dirty state."""

        self.setWindowTitle(
            build_window_title(
                self.current_file_path,
                is_dirty=self.is_document_dirty(),
            )
        )

    def maybe_save_document_changes(self, action_label: str) -> bool:
        """Prompt the user to save dirty changes before a destructive action."""

        if not self.is_document_dirty():
            return True

        reply = QtWidgets.QMessageBox.question(
            self,
            "Unsaved Changes",
            (
                f"Save changes to '{document_display_name(self.current_file_path)}' "
                f"before {action_label}?"
            ),
            QtWidgets.QMessageBox.StandardButton.Save
            | QtWidgets.QMessageBox.StandardButton.Discard
            | QtWidgets.QMessageBox.StandardButton.Cancel,
            QtWidgets.QMessageBox.StandardButton.Save,
        )
        if reply == QtWidgets.QMessageBox.StandardButton.Save:
            return self.save_state_machine()
        return reply == QtWidgets.QMessageBox.StandardButton.Discard
