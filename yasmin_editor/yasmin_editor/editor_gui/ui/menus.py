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

from PyQt5.QtWidgets import QMenu

from yasmin_editor.editor_gui.ui.action_specs import (
    ADD_MENU_ACTIONS,
    EDIT_MENU_ACTIONS,
    FILE_MENU_ACTIONS,
    HELP_MENU_ACTIONS,
    VIEW_MENU_ACTIONS,
)
from yasmin_editor.editor_gui.ui.actions import ensure_action_registry
from yasmin_editor.editor_gui.ui.recent_menu import build_recent_file_menu


def _populate_menu(menu: QMenu, editor, specs) -> None:
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
