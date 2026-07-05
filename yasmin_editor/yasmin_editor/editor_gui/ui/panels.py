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

from yasmin_editor.qt_compat import QtWidgets

from yasmin_editor.editor_gui.ui.canvas_area import build_canvas_frame
from yasmin_editor.editor_gui.ui.metadata import build_metadata_widget
from yasmin_editor.editor_gui.ui.runtime_controls import build_runtime_controls_widget

CANVAS_HEADER_HTML = (
    "<b>Canvas:</b> "
    "<i>Ctrl + double-click nested containers to enter them, drag from blue ports to connect states, "
    "drag a selection box for multi-select, double-click text blocks to edit, scroll to zoom, right-click for options.</i>"
)


def build_right_panel(editor) -> QtWidgets.QWidget:
    """Create the complete right panel."""

    right_panel = QtWidgets.QWidget()
    right_panel.setMinimumWidth(0)
    right_layout = QtWidgets.QVBoxLayout(right_panel)
    right_layout.setContentsMargins(0, 0, 0, 0)
    right_layout.setSpacing(6)

    metadata_widget = build_metadata_widget(editor)
    metadata_widget.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Maximum
    )
    right_layout.addWidget(metadata_widget)

    editor.canvas_header = QtWidgets.QLabel(CANVAS_HEADER_HTML)
    editor.canvas_header.setWordWrap(True)
    editor.canvas_header.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred
    )
    right_layout.addWidget(editor.canvas_header)

    breadcrumb_widget = QtWidgets.QWidget()
    breadcrumb_widget.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Maximum
    )
    editor.breadcrumb_layout = QtWidgets.QHBoxLayout(breadcrumb_widget)
    editor.breadcrumb_layout.setContentsMargins(0, 0, 0, 0)
    right_layout.addWidget(breadcrumb_widget)

    editor.runtime_controls_widget = build_runtime_controls_widget(editor)
    editor.runtime_controls_widget.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Maximum
    )
    right_layout.addWidget(editor.runtime_controls_widget)

    editor.canvas_frame = build_canvas_frame(editor)
    editor.canvas_frame.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding
    )
    right_layout.addWidget(editor.canvas_frame, 1)

    return right_panel
