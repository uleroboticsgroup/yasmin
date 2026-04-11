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

from __future__ import annotations

from PyQt5.QtWidgets import QMessageBox

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

        reply = QMessageBox.question(
            self,
            "Unsaved Changes",
            (
                f"Save changes to '{document_display_name(self.current_file_path)}' "
                f"before {action_label}?"
            ),
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save,
        )
        if reply == QMessageBox.Save:
            return self.save_state_machine()
        return reply == QMessageBox.Discard
