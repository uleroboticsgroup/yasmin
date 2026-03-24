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

from typing import Dict, Optional

from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QTextEdit,
)


class BlackboardKeyDialog(QDialog):
    """Dialog for creating and editing blackboard keys."""

    TYPE_OPTIONS = ["IN", "OUT", "IN/OUT"]
    VALUE_TYPE_OPTIONS = ["str", "int", "float", "bool"]

    def __init__(self, key_data: Optional[Dict[str, str]] = None, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Blackboard Key" if key_data else "Add Blackboard Key")
        self.resize(480, 320)

        key_data = dict(key_data or {})

        layout = QFormLayout(self)

        self.name_edit = QLineEdit(key_data.get("name", ""))
        self.name_edit.setPlaceholderText("Enter key name")
        layout.addRow("Name:*", self.name_edit)

        self.type_combo = QComboBox()
        self.type_combo.addItems(self.TYPE_OPTIONS)
        key_type = key_data.get("key_type", "IN")
        type_index = self.type_combo.findText(key_type)
        if type_index >= 0:
            self.type_combo.setCurrentIndex(type_index)
        layout.addRow("Type:", self.type_combo)

        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(80)
        self.description_edit.setPlainText(key_data.get("description", ""))
        layout.addRow(QLabel("<b>Description (optional):</b>"), self.description_edit)

        self.default_type_combo = QComboBox()
        self.default_type_combo.addItems(self.VALUE_TYPE_OPTIONS)
        default_type = key_data.get("default_type", "str")
        default_type_index = self.default_type_combo.findText(default_type)
        if default_type_index >= 0:
            self.default_type_combo.setCurrentIndex(default_type_index)
        layout.addRow("Default Type:", self.default_type_combo)

        self.default_value_edit = QLineEdit(key_data.get("default_value", ""))
        self.default_value_edit.setPlaceholderText("Optional default value for input keys")
        layout.addRow("Default Value:", self.default_value_edit)

        self.type_combo.currentTextChanged.connect(self._update_default_fields)
        self._update_default_fields(self.type_combo.currentText())

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._accept_with_validation)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _update_default_fields(self, key_type: str) -> None:
        default_allowed = key_type in ("IN", "IN/OUT")
        self.default_type_combo.setEnabled(default_allowed)
        self.default_value_edit.setEnabled(default_allowed)
        if not default_allowed:
            self.default_value_edit.clear()

    def _accept_with_validation(self) -> None:
        if not self.name_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Key name is required!")
            return
        self.accept()

    def get_key_data(self) -> Dict[str, str]:
        key_type = self.type_combo.currentText()
        default_type = self.default_type_combo.currentText() if self.default_type_combo.isEnabled() else ""
        default_value = self.default_value_edit.text().strip() if self.default_value_edit.isEnabled() else ""

        return {
            "name": self.name_edit.text().strip(),
            "key_type": key_type,
            "description": self.description_edit.toPlainText().strip(),
            "default_type": default_type,
            "default_value": default_value,
        }
