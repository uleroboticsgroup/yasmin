#!/usr/bin/env python3
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
"""Shelf split-view widget builders."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas
from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import MIN_CLIPBOARD_DOCK_WIDTH

SHELF_HINT_TEXT = "Drag selected items here to store them temporarily."


def build_clipboard_panel(editor) -> QWidget:
    """Create the shelf side panel."""

    widget = QFrame()
    widget.setObjectName("clipboardPanel")
    widget.setMinimumWidth(MIN_CLIPBOARD_DOCK_WIDTH)
    widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

    layout = QVBoxLayout(widget)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(8)

    header_layout = QHBoxLayout()
    header_layout.addWidget(QLabel("<b>Shelf</b>"))
    header_layout.addStretch()

    editor.clipboard_fit_button = QPushButton("Fit")
    editor.clipboard_fit_button.setToolTip(
        "Recenter and zoom the shelf view without changing the shelf width."
    )
    editor.clipboard_fit_button.clicked.connect(editor.fit_clipboard_panel_to_content)
    header_layout.addWidget(editor.clipboard_fit_button)

    editor.clear_clipboard_button = QPushButton("Clear")
    editor.clear_clipboard_button.setToolTip("Remove everything from the shelf.")
    editor.clear_clipboard_button.clicked.connect(editor.clear_clipboard_contents)
    header_layout.addWidget(editor.clear_clipboard_button)

    editor.clipboard_panel_toggle_button = QPushButton("Hide")
    editor.clipboard_panel_toggle_button.setToolTip("Hide the shelf dock.")
    editor.clipboard_panel_toggle_button.clicked.connect(editor.toggle_clipboard_panel)
    header_layout.addWidget(editor.clipboard_panel_toggle_button)
    layout.addLayout(header_layout)

    hint_label = QLabel(SHELF_HINT_TEXT)
    hint_label.setWordWrap(True)
    hint_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    layout.addWidget(hint_label)

    editor.clipboard_canvas = StateMachineCanvas()
    editor.clipboard_canvas.editor_ref = editor
    editor.clipboard_canvas.layout_sync_handler = editor.sync_clipboard_layout
    editor.clipboard_canvas.setObjectName("clipboardCanvas")
    editor.clipboard_canvas.setToolTip(
        "Selection shelf. Drag selected items here to store them. "
        "Drag stored items back onto the main canvas to place them again."
    )
    editor.clipboard_canvas.setContextMenuPolicy(Qt.NoContextMenu)
    editor.clipboard_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    editor.clipboard_canvas.setMinimumSize(0, 0)
    layout.addWidget(editor.clipboard_canvas, 1)
    return widget
