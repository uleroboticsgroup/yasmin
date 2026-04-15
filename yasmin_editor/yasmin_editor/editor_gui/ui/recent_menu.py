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

"""Qt helpers for the File > Open Recent submenu."""

from __future__ import annotations

import os

from PyQt5.QtWidgets import QAction, QMenu


def build_recent_file_label(index: int, file_path: str) -> str:
    """Return the menu label for one recent document."""

    file_name = os.path.basename(file_path) or file_path
    return f"&{index} {file_name}"


def refresh_recent_file_menu(editor) -> None:
    """Rebuild the editor recent-files submenu from the current store state."""

    menu = getattr(editor, "recent_files_menu", None)
    if menu is None:
        return

    menu.clear()
    recent_files = list(getattr(editor, "recent_files", []))
    if not recent_files:
        empty_action = menu.addAction("(No recent files)")
        empty_action.setEnabled(False)
        return

    for index, file_path in enumerate(recent_files[:9], start=1):
        action = QAction(build_recent_file_label(index, file_path), editor)
        action.setToolTip(file_path)
        action.setStatusTip(file_path)
        action.triggered.connect(
            lambda _checked=False, path=file_path: editor.open_recent_state_machine(path)
        )
        menu.addAction(action)

    if len(recent_files) > 9:
        menu.addSeparator()
        for file_path in recent_files[9:]:
            action = QAction(file_path, editor)
            action.setToolTip(file_path)
            action.setStatusTip(file_path)
            action.triggered.connect(
                lambda _checked=False, path=file_path: editor.open_recent_state_machine(
                    path
                )
            )
            menu.addAction(action)

    menu.addSeparator()
    clear_action = menu.addAction("Clear Recent Files")
    clear_action.triggered.connect(editor.clear_recent_files)


def build_recent_file_menu(editor, file_menu: QMenu) -> QMenu:
    """Create the File > Open Recent submenu and populate its entries."""

    recent_files_menu = file_menu.addMenu("Open Recent")
    editor.recent_files_menu = recent_files_menu
    refresh_recent_file_menu(editor)
    return recent_files_menu
