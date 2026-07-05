# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from typing import Optional, Tuple
from yasmin_editor.qt_compat import QtWidgets


class JoinStateDialog(QtWidgets.QDialog):
    """Dialog for creating/editing a JoinState."""

    def __init__(
        self,
        name: str = "",
        sync_id: str = "",
        outcome: str = "joined",
        description: str = "",
        edit_mode: bool = False,
        parent: Optional[QtWidgets.QDialog] = None,
    ) -> None:
        super().__init__(parent)
        self.edit_mode = edit_mode

        self.setWindowTitle("Edit Join State" if edit_mode else "Add Join State")
        self.resize(400, 300)

        layout = QtWidgets.QFormLayout(self)

        self.name_edit = QtWidgets.QLineEdit(name)
        self.name_edit.setPlaceholderText("Enter join state name (required)")
        layout.addRow("Name:*", self.name_edit)

        self.sync_id_edit = QtWidgets.QLineEdit(sync_id)
        self.sync_id_edit.setPlaceholderText("Optional sync ID (leave empty for default)")
        layout.addRow("Sync ID:", self.sync_id_edit)

        self.outcome_edit = QtWidgets.QLineEdit(outcome)
        self.outcome_edit.setPlaceholderText("Outcome when joined (default: joined)")
        layout.addRow("Outcome:", self.outcome_edit)

        desc_label = QtWidgets.QLabel("<b>Description:</b>")
        self.description_edit = QtWidgets.QTextEdit()
        self.description_edit.setMaximumHeight(80)
        if description:
            self.description_edit.setPlainText(description)
        layout.addRow(desc_label, self.description_edit)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok
            | QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_join_state_data(
        self,
    ) -> Optional[Tuple[str, str, str, str]]:
        """Return (name, sync_id, outcome, description) or None if validation fails."""
        name = self.name_edit.text().strip()
        if not name:
            QtWidgets.QMessageBox.warning(
                self, "Validation Error", "Join state name is required!"
            )
            return None

        sync_id = self.sync_id_edit.text().strip()
        outcome = self.outcome_edit.text().strip() or "joined"
        description = self.description_edit.toPlainText().strip()

        return (name, sync_id, outcome, description)
