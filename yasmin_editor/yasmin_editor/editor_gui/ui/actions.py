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

from yasmin_editor.qt_compat import QAction
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
