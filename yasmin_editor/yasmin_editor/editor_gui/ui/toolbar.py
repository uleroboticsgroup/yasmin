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

from PyQt5.QtWidgets import QAction, QPushButton, QToolBar


def build_toolbar(editor) -> None:
    """Create the main toolbar."""
    toolbar = QToolBar()
    toolbar.setObjectName("yasminEditorToolbar")
    editor.addToolBar(toolbar)

    editor.new_action = QAction("New", editor)
    editor.new_action.setShortcut("Ctrl+N")
    editor.new_action.triggered.connect(editor.new_state_machine)
    toolbar.addAction(editor.new_action)

    editor.open_action = QAction("Open", editor)
    editor.open_action.setShortcut("Ctrl+O")
    editor.open_action.triggered.connect(editor.open_state_machine)
    toolbar.addAction(editor.open_action)

    editor.save_action = QAction("Save", editor)
    editor.save_action.setShortcut("Ctrl+S")
    editor.save_action.triggered.connect(editor.save_state_machine)
    toolbar.addAction(editor.save_action)

    toolbar.addSeparator()

    editor.add_state_action = QAction("Add State", editor)
    editor.add_state_action.triggered.connect(editor.add_state)
    toolbar.addAction(editor.add_state_action)

    editor.add_state_machine_action = QAction("Add State Machine", editor)
    editor.add_state_machine_action.triggered.connect(editor.add_state_machine)
    toolbar.addAction(editor.add_state_machine_action)

    editor.add_concurrence_action = QAction("Add Concurrence", editor)
    editor.add_concurrence_action.triggered.connect(editor.add_concurrence)
    toolbar.addAction(editor.add_concurrence_action)

    editor.add_final_action = QAction("Add Final Outcome", editor)
    editor.add_final_action.triggered.connect(editor.add_final_outcome)
    toolbar.addAction(editor.add_final_action)

    toolbar.addSeparator()

    editor.edit_current_action = QAction("Edit Current Container", editor)
    editor.edit_current_action.triggered.connect(editor.edit_current_container)
    toolbar.addAction(editor.edit_current_action)

    editor.delete_action = QAction("Delete Selected", editor)
    editor.delete_action.triggered.connect(editor.delete_selected)
    toolbar.addAction(editor.delete_action)

    toolbar.addSeparator()

    help_action = QAction("Help", editor)
    help_action.triggered.connect(editor.show_help)
    toolbar.addAction(help_action)

    toolbar.addSeparator()

    editor.runtime_mode_button = QPushButton("Runtime Mode")
    editor.runtime_mode_button.setCheckable(True)
    editor.runtime_mode_button.setToolTip(
        "Enter or leave runtime mode using the current state machine XML snapshot."
    )
    editor.runtime_mode_button.clicked.connect(editor.toggle_runtime_mode)
    toolbar.addWidget(editor.runtime_mode_button)
