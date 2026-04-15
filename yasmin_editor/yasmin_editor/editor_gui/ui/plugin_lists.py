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
"""Qt wiring helpers for the plugin sidebar lists."""

from __future__ import annotations

from collections.abc import Iterator

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem

from yasmin_editor.editor_gui.plugin_catalog import (
    iter_plugin_list_entries,
    list_widget_targets,
    matches_plugin_filter,
)


def _iter_list_items(list_widget: QListWidget) -> Iterator[QListWidgetItem]:
    """Yield all items from a ``QListWidget`` in index order."""

    for index in range(list_widget.count()):
        yield list_widget.item(index)


def populate_plugin_lists(editor) -> None:
    """Populate the left sidebar lists from the plugin manager.

    The display naming and ordering rules live in ``plugin_catalog`` so they can
    be validated without importing Qt.
    """

    for widget_name in list_widget_targets():
        getattr(editor, widget_name).clear()

    for entry in iter_plugin_list_entries(editor.plugin_manager):
        item = QListWidgetItem(entry.display_name)
        item.setData(Qt.UserRole, entry.payload)
        getattr(editor, entry.list_name).addItem(item)


def filter_list_widget(list_widget: QListWidget, text: str) -> None:
    """Apply a case-insensitive text filter to a plugin sidebar list."""

    for item in _iter_list_items(list_widget):
        item.setHidden(not matches_plugin_filter(item.text(), text))
