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

"""Shared dialog primitives for container-style editor dialogs."""

from typing import Dict, List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QComboBox, QDialog, QDialogButtonBox, QFormLayout,
                             QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QMessageBox, QPushButton, QSizePolicy,
                             QTableWidget, QTableWidgetItem, QTextEdit,
                             QVBoxLayout, QWidget)


class ContainerDialogBase(QDialog):
    """Base dialog for state machine and concurrence container editors.

    The derived classes only need to add their container-specific selector row
    and expose a typed ``get_*_data`` method. The shared layout, remapping
    editing, description editing, and validation live here.
    """

    NONE_TEXT = "(None)"

    def __init__(
        self,
        *,
        window_title: str,
        name: str,
        name_placeholder: str,
        outcomes: Optional[List[str]],
        remappings: Optional[Dict[str, str]],
        description: str,
        edit_mode: bool,
        parent: Optional[QDialog] = None,
    ) -> None:
        super().__init__(parent)
        self.edit_mode = edit_mode

        self.setWindowTitle(window_title)
        self.resize(500, 500)

        self.layout = QFormLayout(self)

        self._create_name_row(name, name_placeholder)
        self._create_selector_row()
        self._create_outcomes_row(outcomes or [])
        self._create_description_row(description)
        self._create_remappings_row(remappings or {})
        self._create_button_row()

    def _create_name_row(self, name: str, placeholder: str) -> None:
        """Create the required name input row."""
        self.name_edit = QLineEdit(name)
        self.name_edit.setPlaceholderText(placeholder)
        self.layout.addRow("Name:*", self.name_edit)

    def _create_selector_row(self) -> None:
        """Create the container-specific selector row.

        Derived classes override this to add either a start-state combo box or
        a default-outcome combo box.
        """

    def _create_outcomes_row(self, outcomes: List[str]) -> None:
        """Create the read-only outcomes display row."""
        self.outcomes_label = QLabel("Outcomes:")
        outcomes_text = ", ".join(outcomes)

        self.outcomes_display = QLabel(outcomes_text)
        self.outcomes_display.setWordWrap(True)
        self.outcomes_display.setProperty("infoBox", True)
        self.outcomes_display.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.outcomes_display.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)

        outcome_count = len(outcomes)
        base_height = 24
        extra_height = min(3, outcome_count) * 18
        self.outcomes_display.setMinimumHeight(base_height + extra_height)
        self.outcomes_display.setMaximumHeight(base_height + max(3, outcome_count) * 18)

        self.layout.addRow(self.outcomes_label, self.outcomes_display)

    def _create_description_row(self, description: str) -> None:
        """Create the optional description editor."""
        description_label = QLabel("<b>Description:</b>")
        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(60)

        if description:
            self.description_edit.setPlainText(description)

        self.layout.addRow(description_label, self.description_edit)

    def _create_remappings_row(self, remappings: Dict[str, str]) -> None:
        """Create the remappings editor table and controls."""
        remappings_label = QLabel("<b>Remappings:</b>")
        remappings_widget = QWidget()
        remappings_layout = QVBoxLayout(remappings_widget)
        remappings_layout.setContentsMargins(0, 0, 0, 0)

        self.remappings_table = QTableWidget(0, 2)
        self.remappings_table.setHorizontalHeaderLabels(["Old Key", "New Key"])
        self.remappings_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.remappings_table.setMinimumHeight(80)
        self.remappings_table.setMaximumHeight(150)
        remappings_layout.addWidget(self.remappings_table)

        buttons_layout = QHBoxLayout()

        add_remap_button = QPushButton("Add Row")
        add_remap_button.clicked.connect(self.add_remapping_row)
        buttons_layout.addWidget(add_remap_button)

        remove_remap_button = QPushButton("Remove Row")
        remove_remap_button.clicked.connect(self.remove_remapping_row)
        buttons_layout.addWidget(remove_remap_button)

        buttons_layout.addStretch()
        remappings_layout.addLayout(buttons_layout)

        for old_key, new_key in remappings.items():
            self._add_remapping_row_with_data(old_key, new_key)

        self.layout.addRow(remappings_label, remappings_widget)

    def _create_button_row(self) -> None:
        """Create the standard OK/Cancel button row."""
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.layout.addWidget(buttons)

    def validate_name(self, object_name: str) -> Optional[str]:
        """Return the stripped name or ``None`` if validation fails."""
        name = self.name_edit.text().strip()
        if name:
            return name

        QMessageBox.warning(
            self,
            "Validation Error",
            f"{object_name} name is required!",
        )
        return None

    def parse_outcomes(self) -> List[str]:
        """Return the read-only outcomes as a normalized list."""
        outcomes_text = self.outcomes_display.text().strip()
        return [
            outcome.strip() for outcome in outcomes_text.split(",") if outcome.strip()
        ]

    def parse_remappings(self) -> Dict[str, str]:
        """Return remappings from the table, skipping incomplete rows."""
        remappings: Dict[str, str] = {}
        for row in range(self.remappings_table.rowCount()):
            old_item = self.remappings_table.item(row, 0)
            new_item = self.remappings_table.item(row, 1)
            old_key = old_item.text().strip() if old_item else ""
            new_key = new_item.text().strip() if new_item else ""
            if old_key and new_key:
                remappings[old_key] = new_key
        return remappings

    def parse_description(self) -> str:
        """Return the normalized free-text description."""
        return self.description_edit.toPlainText().strip()

    def create_optional_combo(
        self,
        *,
        values: Optional[List[str]],
        current_value: Optional[str],
        enabled_in_add_mode: bool,
    ) -> QComboBox:
        """Create a combo box with a ``(None)`` sentinel and optional values."""
        combo_box = QComboBox()
        combo_box.addItem(self.NONE_TEXT)

        if not self.edit_mode and not enabled_in_add_mode:
            combo_box.setEnabled(False)

        for value in values or []:
            combo_box.addItem(value)

        if current_value:
            current_index = combo_box.findText(current_value)
            if current_index >= 0:
                combo_box.setCurrentIndex(current_index)

        return combo_box

    @classmethod
    def combo_value_or_none(cls, combo_box: QComboBox) -> Optional[str]:
        """Return the selected combo value or ``None`` for the sentinel item."""
        value = combo_box.currentText()
        return None if value == cls.NONE_TEXT else value

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
        """Add a remapping row pre-filled with values."""
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)
        self.remappings_table.setItem(row, 0, QTableWidgetItem(old_key))
        self.remappings_table.setItem(row, 1, QTableWidgetItem(new_key))
