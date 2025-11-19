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

from typing import Dict, List, Optional, Tuple
from PyQt5.QtWidgets import (
    QLabel,
    QDialog,
    QFormLayout,
    QLineEdit,
    QDialogButtonBox,
    QTextEdit,
    QComboBox,
    QMessageBox,
)
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSizePolicy


class ConcurrenceDialog(QDialog):
    """Dialog for creating/editing Concurrence containers."""

    def __init__(
        self,
        name: str = "",
        outcomes: Optional[List[str]] = None,
        default_outcome: Optional[str] = None,
        remappings: Optional[Dict[str, str]] = None,
        final_outcomes: Optional[List[str]] = None,
        edit_mode: bool = False,
        parent: Optional[QDialog] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Concurrence" if edit_mode else "Add Concurrence")
        self.resize(500, 500)
        self.edit_mode: bool = edit_mode

        layout: QFormLayout = QFormLayout(self)

        # Concurrence name
        self.name_edit: QLineEdit = QLineEdit(name)
        self.name_edit.setPlaceholderText("Enter concurrence name (required)")
        layout.addRow("Name:*", self.name_edit)

        # Default outcome - use combo box
        self.default_outcome_label: QLabel = QLabel("Default Outcome:")
        self.default_outcome_combo: QComboBox = QComboBox()
        self.default_outcome_combo.addItem("(None)")
        if not self.edit_mode:
            self.default_outcome_combo.setEnabled(False)

        # Add available final outcomes to combo
        if final_outcomes:
            for outcome in final_outcomes:
                self.default_outcome_combo.addItem(outcome)

        # Set current default outcome
        if default_outcome:
            index: int = self.default_outcome_combo.findText(default_outcome)
            if index >= 0:
                self.default_outcome_combo.setCurrentIndex(index)

        layout.addRow(self.default_outcome_label, self.default_outcome_combo)

        # Outcomes field (read-only, better representation)
        self.outcomes_label: QLabel = QLabel("Outcomes:")
        outcomes_str: str = ", ".join(outcomes) if outcomes else ""

        self.outcomes_display: QLabel = QLabel(outcomes_str)
        self.outcomes_display.setWordWrap(True)
        self.outcomes_display.setStyleSheet(
            "background: #f0f0f0; border: 1px solid #ccc; padding: 4px;"
        )
        self.outcomes_display.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.outcomes_display.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        num_outcomes: int = len(outcomes_str.split(",")) if outcomes_str else 0
        base_height: int = 24
        extra_height: int = min(3, num_outcomes) * 18
        self.outcomes_display.setMinimumHeight(base_height + extra_height)
        self.outcomes_display.setMaximumHeight(base_height + max(3, num_outcomes) * 18)
        layout.addRow(self.outcomes_label, self.outcomes_display)

        # Remappings
        remappings_label: QLabel = QLabel("<b>Remappings (optional):</b>")
        self.remappings_edit: QTextEdit = QTextEdit()
        self.remappings_edit.setMaximumHeight(100)
        self.remappings_edit.setPlaceholderText(
            "old_key:new_key\nanother_key:another_value"
        )
        if remappings:
            remap_text: str = "\n".join([f"{k}:{v}" for k, v in remappings.items()])
            self.remappings_edit.setPlainText(remap_text)
        layout.addRow(remappings_label, self.remappings_edit)

        # Buttons
        buttons: QDialogButtonBox = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_concurrence_data(self) -> Tuple[str, List[str], str, Dict[str, str]]:
        """Returns: (name, outcomes, default_outcome, remappings)"""
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Validation Error", "Concurrence name is required!")
            return None

        # Parse outcomes from display label (read-only)
        outcomes_text = self.outcomes_display.text().strip()
        outcomes = [o.strip() for o in outcomes_text.split(",") if o.strip()]

        # Get default outcome from combo box
        default_outcome_text = self.default_outcome_combo.currentText()
        default_outcome = (
            None if default_outcome_text == "(None)" else default_outcome_text
        )

        # Parse remappings
        remappings = {}
        remap_text = self.remappings_edit.toPlainText().strip()
        if remap_text:
            for line in remap_text.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    remappings[key.strip()] = value.strip()

        return name, outcomes, default_outcome, remappings
