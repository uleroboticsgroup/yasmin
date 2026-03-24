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
    QComboBox,
    QMessageBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QPushButton,
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
    QTextEdit,
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
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
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

        # Description
        desc_label: QLabel = QLabel("<b>Description (optional):</b>")
        self.description_edit: QTextEdit = QTextEdit()
        self.description_edit.setMaximumHeight(60)
        if description:
            self.description_edit.setPlainText(description)
        layout.addRow(desc_label, self.description_edit)

        # Remappings
        remappings_label: QLabel = QLabel("<b>Remappings (optional):</b>")
        remappings_widget: QWidget = QWidget()
        remappings_layout: QVBoxLayout = QVBoxLayout(remappings_widget)
        remappings_layout.setContentsMargins(0, 0, 0, 0)

        self.remappings_table: QTableWidget = QTableWidget(0, 2)
        self.remappings_table.setHorizontalHeaderLabels(["Old Key", "New Key"])
        self.remappings_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.remappings_table.setMinimumHeight(80)
        self.remappings_table.setMaximumHeight(150)
        remappings_layout.addWidget(self.remappings_table)

        remap_btn_layout: QHBoxLayout = QHBoxLayout()
        add_remap_btn: QPushButton = QPushButton("Add Row")
        add_remap_btn.clicked.connect(self.add_remapping_row)
        remap_btn_layout.addWidget(add_remap_btn)
        remove_remap_btn: QPushButton = QPushButton("Remove Row")
        remove_remap_btn.clicked.connect(self.remove_remapping_row)
        remap_btn_layout.addWidget(remove_remap_btn)
        remap_btn_layout.addStretch()
        remappings_layout.addLayout(remap_btn_layout)

        if remappings:
            for old_key, new_key in remappings.items():
                self._add_remapping_row_with_data(old_key, new_key)

        layout.addRow(remappings_label, remappings_widget)

        # Buttons
        buttons: QDialogButtonBox = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_concurrence_data(
        self,
    ) -> Tuple[str, List[str], str, Dict[str, str], str, List[Dict[str, str]]]:
        """Returns: (name, outcomes, default_outcome, remappings, description, defaults)"""
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

        # Parse remappings from table
        remappings = {}
        for row in range(self.remappings_table.rowCount()):
            old_item = self.remappings_table.item(row, 0)
            new_item = self.remappings_table.item(row, 1)
            old_key = old_item.text().strip() if old_item else ""
            new_key = new_item.text().strip() if new_item else ""
            if old_key and new_key:
                remappings[old_key] = new_key

        description = self.description_edit.toPlainText().strip()

        return name, outcomes, default_outcome, remappings, description, []

    def add_remapping_row(self) -> None:
        """Add an empty row to the remappings table."""
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)

    def remove_remapping_row(self) -> None:
        """Remove the selected row from the remappings table."""
        row = self.remappings_table.currentRow()
        if row >= 0:
            self.remappings_table.removeRow(row)

    def _add_remapping_row_with_data(self, old_key: str, new_key: str) -> None:
        """Add a row to the remappings table pre-filled with data."""
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)
        self.remappings_table.setItem(row, 0, QTableWidgetItem(old_key))
        self.remappings_table.setItem(row, 1, QTableWidgetItem(new_key))
