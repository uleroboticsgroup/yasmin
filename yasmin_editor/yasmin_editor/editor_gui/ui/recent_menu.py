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

import os

from yasmin_editor.qt_compat import QtWidgets, QAction


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


def build_recent_file_menu(editor, file_menu: QtWidgets.QMenu) -> QtWidgets.QMenu:
    """Create the File > Open Recent submenu and populate its entries."""

    recent_files_menu = file_menu.addMenu("Open Recent")
    editor.recent_files_menu = recent_files_menu
    refresh_recent_file_menu(editor)
    return recent_files_menu
