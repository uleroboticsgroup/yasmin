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

from typing import Dict, List, Tuple
from PyQt5.QtWidgets import (
    QLabel,
    QMessageBox,
    QDialog,
    QFormLayout,
    QLineEdit,
    QDialogButtonBox,
    QTextEdit,
)


class ConcurrenceDialog(QDialog):
    """Dialog for creating/editing Concurrence containers."""

    def __init__(
        self,
        name: str = "",
        outcomes: List[str] = None,
        default_outcome: str = None,
        remappings: Dict[str, str] = None,
        edit_mode: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Edit Concurrence" if edit_mode else "Add Concurrence")
        self.resize(500, 500)
        self.edit_mode = edit_mode

        layout = QFormLayout(self)

        # Concurrence name
        self.name_edit = QLineEdit(name)
        self.name_edit.setPlaceholderText("Enter concurrence name (required)")
        layout.addRow("Name:*", self.name_edit)

        # Default outcome
        self.default_outcome_label = QLabel("Default Outcome:*")
        self.default_outcome_edit = QLineEdit()
        self.default_outcome_edit.setPlaceholderText("Enter default outcome name")
        if default_outcome:
            self.default_outcome_edit.setText(default_outcome)
        layout.addRow(self.default_outcome_label, self.default_outcome_edit)

        # Outcomes field
        self.outcomes_label = QLabel("Outcomes (space-separated):")
        self.outcomes_edit = QLineEdit()
        self.outcomes_edit.setPlaceholderText("e.g., outcome1 outcome2 outcome3")
        self.outcomes_edit.setEnabled(False)
        if outcomes:
            self.outcomes_edit.setText(" ".join(outcomes))
        layout.addRow(self.outcomes_label, self.outcomes_edit)

        # Remappings
        remappings_label = QLabel("<b>Remappings (optional):</b>")
        self.remappings_edit = QTextEdit()
        self.remappings_edit.setMaximumHeight(100)
        self.remappings_edit.setPlaceholderText(
            "old_key:new_key\nanother_key:another_value"
        )
        if remappings:
            remap_text = "\n".join([f"{k}:{v}" for k, v in remappings.items()])
            self.remappings_edit.setPlainText(remap_text)
        layout.addRow(remappings_label, self.remappings_edit)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_concurrence_data(self) -> Tuple[str, List[str], str, Dict[str, str]]:
        """Returns: (name, outcomes, default_outcome, remappings)"""
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Validation Error", "Concurrence name is required!")
            return None

        # Parse outcomes
        outcomes_text = self.outcomes_edit.text().strip()
        outcomes = outcomes_text.split()

        # Get default outcome
        default_outcome = self.default_outcome_edit.text().strip()
        if not default_outcome:
            QMessageBox.warning(self, "Validation Error", "Default outcome is required!")
            return None

        # Parse remappings
        remappings = {}
        remap_text = self.remappings_edit.toPlainText().strip()
        if remap_text:
            for line in remap_text.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    remappings[key.strip()] = value.strip()

        return name, outcomes, default_outcome, remappings
