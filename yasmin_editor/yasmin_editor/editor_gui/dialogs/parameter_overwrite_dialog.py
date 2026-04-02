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

from typing import Dict, List, Optional

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


class ParameterOverwriteDialog(QDialog):
    """Dialog for creating and editing a parameter overwrite entry."""

    VALUE_TYPE_OPTIONS = ["", "str", "int", "float", "bool"]

    def __init__(
        self,
        declared_parameters: Optional[List[Dict[str, str]]] = None,
        param_data: Optional[Dict[str, str]] = None,
        parent=None,
        readonly: bool = False,
    ) -> None:
        super().__init__(parent)
        self.readonly = readonly
        self._declared_parameters = list(declared_parameters or [])
        self.setWindowTitle(
            "Edit Parameter Overwrite" if param_data else "Add Parameter Overwrite"
        )
        if self.readonly:
            self.setWindowTitle(self.windowTitle() + " (Readonly)")
        self.resize(480, 360)

        param_data = dict(param_data or {})
        layout = QFormLayout(self)

        self.name_edit = QLineEdit(param_data.get("name", ""))
        self.name_edit.setPlaceholderText("Enter parent parameter name")
        self.name_edit.setReadOnly(self.readonly)
        layout.addRow("Name:*", self.name_edit)

        self.child_param_combo = QComboBox()
        for entry in self._declared_parameters:
            param_name = str(entry.get("name", "") or "").strip()
            if not param_name:
                continue
            description = str(entry.get("description", "") or "").strip()
            label = f"{param_name}: {description}" if description else param_name
            self.child_param_combo.addItem(label, param_name)
        self.child_param_combo.setEnabled(not self.readonly)
        current_child_param = str(param_data.get("child_parameter", "") or "").strip()
        current_index = self.child_param_combo.findData(current_child_param)
        if current_index >= 0:
            self.child_param_combo.setCurrentIndex(current_index)
        layout.addRow("Overrides:*", self.child_param_combo)

        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(80)
        self.description_edit.setPlainText(param_data.get("description", ""))
        self.description_edit.setReadOnly(self.readonly)
        layout.addRow(QLabel("<b>Description:</b>"), self.description_edit)

        self.default_type_combo = QComboBox()
        self.default_type_combo.addItem("No default", "")
        for option in self.VALUE_TYPE_OPTIONS[1:]:
            self.default_type_combo.addItem(option, option)
        default_type = str(param_data.get("default_type", "") or "")
        default_type_index = self.default_type_combo.findData(default_type)
        if default_type_index >= 0:
            self.default_type_combo.setCurrentIndex(default_type_index)
        self.default_type_combo.currentIndexChanged.connect(
            self._update_default_value_state
        )
        self.default_type_combo.setEnabled(not self.readonly)
        layout.addRow("Default Type:", self.default_type_combo)

        self.default_value_edit = QLineEdit(
            str(param_data.get("default_value", "") or "")
        )
        self.default_value_edit.setReadOnly(self.readonly)
        self.default_value_edit.setPlaceholderText(
            "Default value. Leave empty for an empty string when type is str"
        )
        layout.addRow("Default Value:", self.default_value_edit)
        self._update_default_value_state()

        buttons = QDialogButtonBox(
            QDialogButtonBox.Close
            if self.readonly
            else (QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        )
        if self.readonly:
            buttons.rejected.connect(self.reject)
        else:
            buttons.accepted.connect(self._accept_with_validation)
            buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _update_default_value_state(self) -> None:
        has_default_type = bool(self.default_type_combo.currentData())
        self.default_value_edit.setEnabled(
            self.default_type_combo.isEnabled() and has_default_type
        )

    def _accept_with_validation(self) -> None:
        if not self.name_edit.text().strip():
            QMessageBox.warning(
                self,
                "Validation Error",
                "Parameter name is required!",
            )
            return
        if (
            self.child_param_combo.currentIndex() < 0
            or not self.child_param_combo.currentData()
        ):
            QMessageBox.warning(
                self,
                "Validation Error",
                "Please select a declared child parameter to override.",
            )
            return
        self.accept()

    def get_parameter_data(self) -> Dict[str, str]:
        default_type = str(self.default_type_combo.currentData() or "")
        default_value = self.default_value_edit.text().strip() if default_type else ""
        return {
            "name": self.name_edit.text().strip(),
            "child_parameter": str(self.child_param_combo.currentData() or "").strip(),
            "description": self.description_edit.toPlainText().strip(),
            "default_type": default_type,
            "default_value": default_value,
        }
