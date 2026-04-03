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

from PyQt5.QtWidgets import QFrame, QVBoxLayout

from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas


def build_canvas_frame(editor) -> QFrame:
    """Create the canvas frame and canvas widget."""
    frame = QFrame()
    editor.canvas_frame_layout = QVBoxLayout(frame)
    editor.canvas_frame_layout.setContentsMargins(0, 0, 0, 0)

    editor.canvas = StateMachineCanvas()
    editor.canvas.editor_ref = editor
    editor.canvas.scene.selectionChanged.connect(editor.refresh_visual_highlighting)
    editor.canvas_frame_layout.addWidget(editor.canvas)

    return frame
