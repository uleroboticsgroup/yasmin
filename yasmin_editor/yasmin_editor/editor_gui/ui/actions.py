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

"""Shared QAction registry used by the toolbar and menu bar."""

from PyQt5.QtWidgets import QAction

from yasmin_editor.editor_gui.ui.action_specs import ALL_ACTION_GROUPS, ActionSpec


class EditorActionRegistry:
    """Create and cache shared window actions.

    The same QAction instances are reused by the toolbar and the menu bar so the
    shortcuts, enabled states and checked states remain synchronized.
    """

    def __init__(self, editor) -> None:
        self.editor = editor
        self._actions: dict[str, QAction] = {}

    def get(self, spec: ActionSpec) -> QAction:
        action = self._actions.get(spec.attribute_name)
        if action is not None:
            return action

        action = QAction(spec.text, self.editor)
        if spec.shortcut:
            action.setShortcut(spec.shortcut)
        if spec.checkable:
            action.setCheckable(True)
        if spec.tool_tip:
            action.setToolTip(spec.tool_tip)
        if spec.status_tip:
            action.setStatusTip(spec.status_tip)
        action.triggered.connect(getattr(self.editor, spec.callback_name))

        self._actions[spec.attribute_name] = action
        setattr(self.editor, spec.attribute_name, action)
        return action

    def preload_all(self) -> None:
        for group in ALL_ACTION_GROUPS:
            for spec in group:
                self.get(spec)


def ensure_action_registry(editor) -> EditorActionRegistry:
    registry = getattr(editor, "_action_registry", None)
    if registry is not None:
        return registry

    registry = EditorActionRegistry(editor)
    registry.preload_all()
    editor._action_registry = registry
    return registry
