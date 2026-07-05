# Copyright (C) 2026 Maik Knof
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

from typing import Optional
from yasmin_editor.qt_compat import QtWidgets


class OutcomeDescriptionDialog(QtWidgets.QDialog):
    """Dialog for editing a final outcome."""

    def __init__(
        self,
        outcome_name: str,
        description: str = "",
        parent: Optional[QtWidgets.QDialog] = None,
        readonly: bool = False,
    ) -> None:
        super().__init__(parent)
        self.readonly = readonly
        self.setWindowTitle("Edit Outcome" + (" (Readonly)" if self.readonly else ""))
        self.resize(500, 260)

        layout = QtWidgets.QFormLayout(self)

        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setText(outcome_name)
        self.name_edit.setReadOnly(self.readonly)
        layout.addRow("Outcome:", self.name_edit)

        self.description_edit = QtWidgets.QTextEdit()
        self.description_edit.setPlainText(description)
        self.description_edit.setReadOnly(self.readonly)
        layout.addRow("Description:", self.description_edit)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Close
            if self.readonly
            else (
                QtWidgets.QDialogButtonBox.StandardButton.Ok
                | QtWidgets.QDialogButtonBox.StandardButton.Cancel
            )
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
