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

from typing import Dict, List, Optional
from yasmin_editor.qt_compat import QtWidgets


class ParameterOverwriteDialog(QtWidgets.QDialog):
    """Dialog for creating and editing a parameter overwrite entry."""

    VALUE_TYPE_OPTIONS = [
        "",
        "str",
        "int",
        "float",
        "bool",
        "list[str]",
        "list[int]",
        "list[float]",
        "list[bool]",
        "dict[str,str]",
        "dict[str,int]",
        "dict[str,float]",
        "dict[str,bool]",
    ]

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
        layout = QtWidgets.QFormLayout(self)

        self.name_edit = QtWidgets.QLineEdit(param_data.get("name", ""))
        self.name_edit.setPlaceholderText("Enter parent parameter name")
        self.name_edit.setReadOnly(self.readonly)
        layout.addRow("Name:*", self.name_edit)

        self.child_param_combo = QtWidgets.QComboBox()
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

        self.description_edit = QtWidgets.QTextEdit()
        self.description_edit.setMaximumHeight(80)
        self.description_edit.setPlainText(param_data.get("description", ""))
        self.description_edit.setReadOnly(self.readonly)
        layout.addRow(QtWidgets.QLabel("<b>Description:</b>"), self.description_edit)

        self.default_type_combo = QtWidgets.QComboBox()
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

        self.default_value_edit = QtWidgets.QLineEdit(
            str(param_data.get("default_value", "") or "")
        )
        self.default_value_edit.setReadOnly(self.readonly)
        self.default_value_edit.setPlaceholderText(
            'Default value. Use JSON for list/dict types, e.g. [1, 2] or {"foo": 1}'
        )
        self.default_value_edit.setToolTip(
            "Scalar defaults are entered directly. Container defaults use JSON syntax "
            'with homogeneous element/value types, e.g. [1, 2, 3] or {"foo": true}.'
        )
        layout.addRow("Default Value:", self.default_value_edit)
        self._update_default_value_state()

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
            QtWidgets.QMessageBox.warning(
                self,
                "Validation Error",
                "Parameter name is required!",
            )
            return
        if (
            self.child_param_combo.currentIndex() < 0
            or not self.child_param_combo.currentData()
        ):
            QtWidgets.QMessageBox.warning(
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
