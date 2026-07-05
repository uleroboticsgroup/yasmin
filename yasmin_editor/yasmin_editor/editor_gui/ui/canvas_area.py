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
from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas


def build_canvas_frame(editor) -> QtWidgets.QFrame:
    """Create the canvas frame and canvas widget."""
    frame = QtWidgets.QFrame()
    frame.setMinimumSize(0, 0)
    editor.canvas_frame_layout = QtWidgets.QVBoxLayout(frame)
    editor.canvas_frame_layout.setContentsMargins(0, 0, 0, 0)

    editor.canvas = StateMachineCanvas()
    editor.canvas.setMinimumSize(0, 0)
    editor.canvas.editor_ref = editor
    editor.canvas.layout_sync_handler = (
        editor.sync_current_container_layout_and_record_history
    )
    editor.canvas.scene.selectionChanged.connect(editor.on_canvas_selection_changed)
    editor.canvas_frame_layout.addWidget(editor.canvas)

    return frame
