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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMenu, QToolBar, QToolButton

from yasmin_editor.editor_gui.ui.action_specs import (
    HELP_MENU_ACTIONS,
    PRIMARY_TOOLBAR_ACTIONS,
)
from yasmin_editor.editor_gui.ui.actions import ensure_action_registry
from yasmin_editor.editor_gui.ui.toolbar_config import (
    ADD_TOOLBAR_MENU,
    SELECTION_TOOLBAR_MENU,
    ToolbarMenuSpec,
)


def _build_menu_button(editor, toolbar: QToolBar, spec: ToolbarMenuSpec) -> None:
    registry = ensure_action_registry(editor)

    button = QToolButton(toolbar)
    button.setObjectName(spec.object_name)
    setattr(editor, spec.button_attribute_name, button)
    button.setText(spec.text)
    button.setToolTip(spec.tool_tip)
    button.setPopupMode(QToolButton.InstantPopup)
    button.setToolButtonStyle(Qt.ToolButtonTextOnly)

    menu = QMenu(button)
    for attribute_name in spec.action_attributes:
        menu.addAction(getattr(editor, attribute_name))
    button.setMenu(menu)

    toolbar.addWidget(button)


def _add_runtime_mode_button(toolbar: QToolBar, editor) -> None:
    editor.runtime_mode_button = QToolButton(toolbar)
    editor.runtime_mode_button.setObjectName("runtimeModeButton")
    editor.runtime_mode_button.setText("Runtime Mode")
    editor.runtime_mode_button.setCheckable(True)
    editor.runtime_mode_button.setToolButtonStyle(Qt.ToolButtonTextOnly)
    editor.runtime_mode_button.setToolTip(
        "Enter or leave runtime mode using the current state machine XML snapshot."
    )
    editor.runtime_mode_button.clicked.connect(editor.toggle_runtime_mode)
    toolbar.addWidget(editor.runtime_mode_button)


def build_toolbar(editor) -> None:
    """Create the main toolbar.

    The toolbar stays compact and keeps the original primary actions visible,
    while less frequent add and selection workflows move into dedicated
    drop-down buttons.
    """

    toolbar = QToolBar()
    toolbar.setObjectName("yasminEditorToolbar")
    toolbar.setMovable(False)
    toolbar.setFloatable(False)
    toolbar.setToolButtonStyle(Qt.ToolButtonTextOnly)
    editor.addToolBar(toolbar)

    registry = ensure_action_registry(editor)
    for spec in PRIMARY_TOOLBAR_ACTIONS:
        if spec.separator_before:
            toolbar.addSeparator()
        toolbar.addAction(registry.get(spec))
        if spec.attribute_name == "add_state_action":
            _build_menu_button(editor, toolbar, ADD_TOOLBAR_MENU)
        if spec.attribute_name == "delete_action":
            _build_menu_button(editor, toolbar, SELECTION_TOOLBAR_MENU)

    toolbar.addSeparator()
    toolbar.addAction(registry.get(HELP_MENU_ACTIONS[0]))

    toolbar.addSeparator()
    _add_runtime_mode_button(toolbar, editor)
