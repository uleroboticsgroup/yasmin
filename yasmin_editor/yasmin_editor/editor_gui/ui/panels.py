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

from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from yasmin_editor.editor_gui.ui.canvas_area import build_canvas_frame
from yasmin_editor.editor_gui.ui.metadata import build_metadata_widget
from yasmin_editor.editor_gui.ui.runtime_controls import build_runtime_controls_widget
from yasmin_editor.editor_gui.ui.sidebars import build_left_panel


def build_right_panel(editor) -> QWidget:
    """Create the complete right panel."""
    right_panel = QWidget()
    right_layout = QVBoxLayout(right_panel)

    metadata_widget = build_metadata_widget(editor)
    right_layout.addWidget(metadata_widget)

    editor.canvas_header = QLabel(
        "<b>State Machine Canvas:</b> "
        "<i>(Ctrl + double-click a nested container to enter it, drag from blue port to create transitions, double-click text blocks to edit inline, scroll to zoom, right-click for options)</i>"
    )
    right_layout.addWidget(editor.canvas_header)

    breadcrumb_widget = QWidget()
    editor.breadcrumb_layout = QHBoxLayout(breadcrumb_widget)
    editor.breadcrumb_layout.setContentsMargins(0, 0, 0, 0)
    right_layout.addWidget(breadcrumb_widget)

    editor.runtime_controls_widget = build_runtime_controls_widget(editor)
    right_layout.addWidget(editor.runtime_controls_widget)

    editor.canvas_frame = build_canvas_frame(editor)
    right_layout.addWidget(editor.canvas_frame)

    return right_panel
