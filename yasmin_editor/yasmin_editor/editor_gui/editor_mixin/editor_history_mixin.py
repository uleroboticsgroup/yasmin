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

from copy import deepcopy

from yasmin_editor.editor_gui.history import (
    EditorHistory,
    EditorHistorySnapshot,
    container_path_names,
    resolve_container_path,
)


class EditorHistoryMixin:
    """Mixin that provides snapshot-based undo/redo support for the editor."""

    def _create_history(self) -> None:
        self.history = EditorHistory()
        self._history_restore_active = False

    def create_history_snapshot(self) -> EditorHistorySnapshot:
        self.sync_current_container_layout()
        return EditorHistorySnapshot(
            root_model=deepcopy(self.root_model),
            container_path=container_path_names(self.current_container_path),
            current_file_path=self.current_file_path,
        )

    def reset_history(self) -> None:
        if not hasattr(self, "history"):
            return
        self.history.reset(self.create_history_snapshot())
        self.reset_document_dirty_state()
        self.update_history_actions()

    def record_history_checkpoint(self) -> bool:
        if not hasattr(self, "history") or self._history_restore_active:
            return False
        changed = self.history.record(self.create_history_snapshot())
        self.update_history_actions()
        self.update_document_window_title()
        return changed

    def sync_current_container_layout_and_record_history(self) -> None:
        self.sync_current_container_layout()
        self.record_history_checkpoint()

    def update_history_actions(self) -> None:
        history = getattr(self, "history", None)
        read_only = (
            self.is_read_only_mode() if hasattr(self, "is_read_only_mode") else False
        )
        action_states = {
            "undo_action": bool(history and history.can_undo and not read_only),
            "redo_action": bool(history and history.can_redo and not read_only),
        }
        for action_name, enabled in action_states.items():
            action = getattr(self, action_name, None)
            if action is not None:
                action.setEnabled(enabled)

    def restore_history_snapshot(self, snapshot: EditorHistorySnapshot) -> None:
        self._history_restore_active = True
        try:
            model = deepcopy(snapshot.root_model)
            self._clear_external_xml_view()
            self.reset_editor_state(model)
            self.current_file_path = snapshot.current_file_path
            self.current_container_path = resolve_container_path(
                model, snapshot.container_path
            )
            self.current_runtime_container_path = list(snapshot.container_path)
            self.render_current_container(fit_view=True)
        finally:
            self._history_restore_active = False
        self.update_history_actions()
        self.update_document_window_title()

    def undo_last_action(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        if not hasattr(self, "history"):
            return

        snapshot = self.history.undo(self.create_history_snapshot())
        if snapshot is None:
            self.statusBar().showMessage("Nothing to undo", 2000)
            self.update_history_actions()
            return

        self.restore_history_snapshot(snapshot)
        self.statusBar().showMessage("Undid last change", 2000)

    def redo_last_action(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        if not hasattr(self, "history"):
            return

        snapshot = self.history.redo(self.create_history_snapshot())
        if snapshot is None:
            self.statusBar().showMessage("Nothing to redo", 2000)
            self.update_history_actions()
            return

        self.restore_history_snapshot(snapshot)
        self.statusBar().showMessage("Redid last change", 2000)
