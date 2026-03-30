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

from typing import Optional

from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLineEdit,
    QTextEdit,
)


class OutcomeDescriptionDialog(QDialog):
    """Dialog for editing a final outcome."""

    def __init__(
        self,
        outcome_name: str,
        description: str = "",
        parent: Optional[QDialog] = None,
        readonly: bool = False,
    ) -> None:
        super().__init__(parent)
        self.readonly = readonly
        self.setWindowTitle("Edit Outcome" + (" (Readonly)" if self.readonly else ""))
        self.resize(500, 260)

        layout = QFormLayout(self)

        self.name_edit = QLineEdit()
        self.name_edit.setText(outcome_name)
        self.name_edit.setReadOnly(self.readonly)
        layout.addRow("Outcome:", self.name_edit)

        self.description_edit = QTextEdit()
        self.description_edit.setPlainText(description)
        self.description_edit.setReadOnly(self.readonly)
        layout.addRow("Description:", self.description_edit)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Close
            if self.readonly
            else (QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        )
        if self.readonly:
            buttons.rejected.connect(self.reject)
        else:
            buttons.accepted.connect(self.accept)
            buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_outcome_name(self) -> str:
        return self.name_edit.text().strip()

    def get_description(self) -> str:
        return self.description_edit.toPlainText().strip()
