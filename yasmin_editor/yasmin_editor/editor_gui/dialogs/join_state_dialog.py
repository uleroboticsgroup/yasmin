# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from typing import Optional, Tuple

from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QTextEdit,
)


class JoinStateDialog(QDialog):
    """Dialog for creating/editing a JoinState."""

    def __init__(
        self,
        name: str = "",
        sync_id: str = "",
        outcome: str = "joined",
        description: str = "",
        edit_mode: bool = False,
        parent: Optional[QDialog] = None,
    ) -> None:
        super().__init__(parent)
        self.edit_mode = edit_mode

        self.setWindowTitle("Edit Join State" if edit_mode else "Add Join State")
        self.resize(400, 300)

        layout = QFormLayout(self)

        self.name_edit = QLineEdit(name)
        self.name_edit.setPlaceholderText("Enter join state name (required)")
        layout.addRow("Name:*", self.name_edit)

        self.sync_id_edit = QLineEdit(sync_id)
        self.sync_id_edit.setPlaceholderText("Optional sync ID (leave empty for default)")
        layout.addRow("Sync ID:", self.sync_id_edit)

        self.outcome_edit = QLineEdit(outcome)
        self.outcome_edit.setPlaceholderText("Outcome when joined (default: joined)")
        layout.addRow("Outcome:", self.outcome_edit)

        desc_label = QLabel("<b>Description:</b>")
        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(80)
        if description:
            self.description_edit.setPlainText(description)
        layout.addRow(desc_label, self.description_edit)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_join_state_data(
        self,
    ) -> Optional[Tuple[str, str, str, str]]:
        """Return (name, sync_id, outcome, description) or None if validation fails."""
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Validation Error", "Join state name is required!")
            return None

        sync_id = self.sync_id_edit.text().strip()
        outcome = self.outcome_edit.text().strip() or "joined"
        description = self.description_edit.toPlainText().strip()

        return (name, sync_id, outcome, description)
