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

from collections.abc import Iterator

from yasmin_editor.qt_compat import Qt, QtWidgets

from yasmin_editor.editor_gui.plugin_catalog import (
    iter_plugin_list_entries,
    list_widget_targets,
    matches_plugin_filter,
)


def _iter_list_items(
    list_widget: QtWidgets.QListWidget,
) -> Iterator[QtWidgets.QListWidgetItem]:
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
        item = QtWidgets.QListWidgetItem(entry.display_name)
        item.setData(Qt.ItemDataRole.UserRole, entry.payload)
        getattr(editor, entry.list_name).addItem(item)


def filter_list_widget(list_widget: QtWidgets.QListWidget, text: str) -> None:
    """Apply a case-insensitive text filter to a plugin sidebar list."""

    for item in _iter_list_items(list_widget):
        item.setHidden(not matches_plugin_filter(item.text(), text))
