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
from yasmin_editor.editor_gui.ui.action_specs import (
    ADD_MENU_ACTIONS,
    EDIT_MENU_ACTIONS,
    FILE_MENU_ACTIONS,
    HELP_MENU_ACTIONS,
    VIEW_MENU_ACTIONS,
)
from yasmin_editor.editor_gui.ui.actions import ensure_action_registry
from yasmin_editor.editor_gui.ui.recent_menu import build_recent_file_menu


def _populate_menu(menu: QtWidgets.QMenu, editor, specs) -> None:
    registry = ensure_action_registry(editor)
    first_item = True
    for spec in specs:
        if spec.separator_before and not first_item:
            menu.addSeparator()
        menu.addAction(registry.get(spec))
        first_item = False


def build_menu_bar(editor) -> None:
    """Create the application menu bar.

    The menu bar keeps the toolbar close to the original editor layout while the
    newer shelf and extraction workflows remain available through menus and
    shortcuts.
    """

    menu_bar = editor.menuBar()
    menu_bar.clear()

    file_menu = menu_bar.addMenu("File")
    _populate_menu(file_menu, editor, FILE_MENU_ACTIONS)
    file_menu.addSeparator()
    build_recent_file_menu(editor, file_menu)

    add_menu = menu_bar.addMenu("Add")
    _populate_menu(add_menu, editor, ADD_MENU_ACTIONS)

    edit_menu = menu_bar.addMenu("Edit")
    _populate_menu(edit_menu, editor, EDIT_MENU_ACTIONS)

    view_menu = menu_bar.addMenu("View")
    _populate_menu(view_menu, editor, VIEW_MENU_ACTIONS)

    help_menu = menu_bar.addMenu("Help")
    _populate_menu(help_menu, editor, HELP_MENU_ACTIONS)
