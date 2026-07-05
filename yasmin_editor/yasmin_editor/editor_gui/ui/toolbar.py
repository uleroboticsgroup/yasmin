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

from yasmin_editor.qt_compat import Qt, QtWidgets

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


def _build_menu_button(
    editor, toolbar: QtWidgets.QToolBar, spec: ToolbarMenuSpec
) -> None:
    registry = ensure_action_registry(editor)

    button = QtWidgets.QToolButton(toolbar)
    button.setObjectName(spec.object_name)
    setattr(editor, spec.button_attribute_name, button)
    button.setText(spec.text)
    button.setToolTip(spec.tool_tip)
    button.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
    button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)

    menu = QtWidgets.QMenu(button)
    for attribute_name in spec.action_attributes:
        menu.addAction(getattr(editor, attribute_name))
    button.setMenu(menu)

    toolbar.addWidget(button)


def _add_runtime_mode_button(toolbar: QtWidgets.QToolBar, editor) -> None:
    editor.runtime_mode_button = QtWidgets.QToolButton(toolbar)
    editor.runtime_mode_button.setObjectName("runtimeModeButton")
    editor.runtime_mode_button.setText("Runtime Mode")
    editor.runtime_mode_button.setCheckable(True)
    editor.runtime_mode_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
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

    toolbar = QtWidgets.QToolBar()
    toolbar.setObjectName("yasminEditorToolbar")
    toolbar.setMovable(False)
    toolbar.setFloatable(False)
    toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
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
