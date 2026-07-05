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

from yasmin_editor.qt_compat import Qt, QtWidgets
from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas
from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import MIN_CLIPBOARD_DOCK_WIDTH

SHELF_HINT_TEXT = "Drag selected items here to store them temporarily."


def build_clipboard_panel(editor) -> QtWidgets.QWidget:
    """Create the shelf side panel."""

    widget = QtWidgets.QFrame()
    widget.setObjectName("clipboardPanel")
    widget.setMinimumWidth(MIN_CLIPBOARD_DOCK_WIDTH)
    widget.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Expanding
    )

    layout = QtWidgets.QVBoxLayout(widget)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(8)

    header_layout = QtWidgets.QHBoxLayout()
    header_layout.addWidget(QtWidgets.QLabel("<b>Shelf</b>"))
    header_layout.addStretch()

    editor.clipboard_fit_button = QtWidgets.QPushButton("Fit")
    editor.clipboard_fit_button.setObjectName("clipboardFitButton")
    editor.clipboard_fit_button.setToolTip(
        "Recenter and zoom the shelf view without changing the shelf width."
    )
    editor.clipboard_fit_button.clicked.connect(editor.fit_clipboard_panel_to_content)
    header_layout.addWidget(editor.clipboard_fit_button)

    editor.clear_clipboard_button = QtWidgets.QPushButton("Clear")
    editor.clear_clipboard_button.setObjectName("clearClipboardButton")
    editor.clear_clipboard_button.setToolTip("Remove everything from the shelf.")
    editor.clear_clipboard_button.clicked.connect(editor.clear_clipboard_contents)
    header_layout.addWidget(editor.clear_clipboard_button)

    editor.clipboard_panel_toggle_button = QtWidgets.QPushButton("Hide")
    editor.clipboard_panel_toggle_button.setObjectName("clipboardPanelToggleButton")
    editor.clipboard_panel_toggle_button.setToolTip("Hide the shelf dock.")
    editor.clipboard_panel_toggle_button.clicked.connect(editor.toggle_clipboard_panel)
    header_layout.addWidget(editor.clipboard_panel_toggle_button)
    layout.addLayout(header_layout)

    hint_label = QtWidgets.QLabel(SHELF_HINT_TEXT)
    hint_label.setObjectName("clipboardHintLabel")
    hint_label.setWordWrap(True)
    hint_label.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred
    )
    layout.addWidget(hint_label)

    editor.clipboard_canvas = StateMachineCanvas()
    editor.clipboard_canvas.editor_ref = editor
    editor.clipboard_canvas.layout_sync_handler = editor.sync_clipboard_layout
    editor.clipboard_canvas.setObjectName("clipboardCanvas")
    editor.clipboard_canvas.setToolTip(
        "Selection shelf. Drag selected items here to store them. "
        "Drag stored items back onto the main canvas to place them again."
    )
    editor.clipboard_canvas.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
    editor.clipboard_canvas.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding
    )
    editor.clipboard_canvas.setMinimumSize(0, 0)
    layout.addWidget(editor.clipboard_canvas, 1)
    return widget
