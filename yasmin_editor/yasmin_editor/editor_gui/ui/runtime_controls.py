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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget



def build_runtime_controls_widget(editor) -> QWidget:
    """Create the runtime control bar."""
    widget = QWidget()
    editor.runtime_controls_layout = QHBoxLayout(widget)
    editor.runtime_controls_layout.setContentsMargins(0, 0, 0, 0)

    editor.runtime_status_label = QLabel("Ready")
    editor.runtime_status_label.setAlignment(Qt.AlignCenter)
    editor.runtime_status_label.setMinimumWidth(120)
    editor.runtime_status_label.setTextFormat(Qt.PlainText)
    editor.runtime_controls_layout.addWidget(editor.runtime_status_label)

    editor.runtime_play_button = QPushButton("Play")
    editor.runtime_play_button.setToolTip(
        "Start the runtime or resume execution after a pause."
    )
    editor.runtime_play_button.clicked.connect(editor.on_runtime_play_clicked)
    editor.runtime_controls_layout.addWidget(editor.runtime_play_button)

    editor.runtime_pause_button = QPushButton("Pause")
    editor.runtime_pause_button.setToolTip(
        "Pause execution at the next state boundary."
    )
    editor.runtime_pause_button.clicked.connect(editor.on_runtime_pause_clicked)
    editor.runtime_controls_layout.addWidget(editor.runtime_pause_button)

    editor.runtime_step_button = QPushButton("Play Once")
    editor.runtime_step_button.setToolTip(
        "Execute exactly one state and pause before the following state starts."
    )
    editor.runtime_step_button.clicked.connect(editor.on_runtime_step_clicked)
    editor.runtime_controls_layout.addWidget(editor.runtime_step_button)

    editor.runtime_cancel_state_button = QPushButton("Cancel State")
    editor.runtime_cancel_state_button.setToolTip(
        "Request cancellation of the currently active state."
    )
    editor.runtime_cancel_state_button.clicked.connect(
        editor.on_runtime_cancel_state_clicked
    )
    editor.runtime_controls_layout.addWidget(editor.runtime_cancel_state_button)

    editor.runtime_cancel_sm_button = QPushButton("Cancel State Machine")
    editor.runtime_cancel_sm_button.setToolTip(
        "Request cancellation of the complete runtime state machine."
    )
    editor.runtime_cancel_sm_button.clicked.connect(editor.on_runtime_cancel_sm_clicked)
    editor.runtime_controls_layout.addWidget(editor.runtime_cancel_sm_button)

    editor.runtime_restart_button = QPushButton("Restart")
    editor.runtime_restart_button.setToolTip(
        "Recreate the runtime state machine from a fresh XML snapshot."
    )
    editor.runtime_restart_button.clicked.connect(editor.restart_runtime_mode)
    editor.runtime_controls_layout.addWidget(editor.runtime_restart_button)

    editor.runtime_controls_layout.addStretch()
    return widget
