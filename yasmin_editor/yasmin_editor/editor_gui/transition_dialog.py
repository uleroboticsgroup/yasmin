# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import List, Tuple
from PyQt5.QtWidgets import (
    QLabel,
    QMessageBox,
    QDialog,
    QFormLayout,
    QComboBox,
    QDialogButtonBox,
)

from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode


class TransitionDialog(QDialog):
    """Dialog for creating transitions between states."""

    def __init__(self, from_state, available_targets: List, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Transition")
        self.resize(400, 200)

        layout = QFormLayout(self)

        # From state
        from_label = QLabel(f"<b>{from_state.name}</b>")
        layout.addRow("From State:", from_label)

        # Outcome
        self.outcome_combo = QComboBox()

        # Get used outcomes to filter them out
        used_outcomes = (
            from_state.get_used_outcomes()
            if hasattr(from_state, "get_used_outcomes")
            else set()
        )

        available_outcomes = []
        if hasattr(from_state, "plugin_info") and from_state.plugin_info:
            available_outcomes = [
                o for o in from_state.plugin_info.outcomes if o not in used_outcomes
            ]

        if not available_outcomes:
            QMessageBox.warning(
                self,
                "Error",
                "This state has no available outcomes (all outcomes already used)!",
            )
        else:
            for outcome in available_outcomes:
                self.outcome_combo.addItem(outcome)

        layout.addRow("Outcome:", self.outcome_combo)

        # To state
        self.target_combo = QComboBox()
        for target in available_targets:
            if isinstance(target, (StateNode, ContainerStateNode)):
                self.target_combo.addItem(target.name, target)
            elif isinstance(target, FinalOutcomeNode):
                self.target_combo.addItem(f"[Final] {target.name}", target)
        layout.addRow("To State:", self.target_combo)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_transition_data(self) -> Tuple[str, object]:
        outcome = self.outcome_combo.currentText()
        target = self.target_combo.currentData()
        return outcome, target
