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
"""Dock builders for secondary editor panels."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDockWidget, QSizePolicy, QWidget

from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import MIN_CLIPBOARD_DOCK_WIDTH
from yasmin_editor.editor_gui.ui.clipboard_panel import build_clipboard_panel


def build_clipboard_dock(editor) -> QDockWidget:
    """Create the hidden right-side shelf dock."""

    clipboard_panel = build_clipboard_panel(editor)

    clipboard_dock = QDockWidget("Shelf", editor)
    clipboard_dock.setObjectName("clipboardDock")
    clipboard_dock.setAllowedAreas(Qt.RightDockWidgetArea)
    clipboard_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
    clipboard_dock.setTitleBarWidget(QWidget())
    clipboard_dock.setWidget(clipboard_panel)
    clipboard_dock.setMinimumWidth(MIN_CLIPBOARD_DOCK_WIDTH)
    clipboard_dock.setMaximumWidth(16777215)
    clipboard_dock.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
    clipboard_dock.hide()

    editor.clipboard_panel = clipboard_panel
    editor.clipboard_dock = clipboard_dock
    editor.addDockWidget(Qt.RightDockWidgetArea, clipboard_dock)

    return clipboard_dock
