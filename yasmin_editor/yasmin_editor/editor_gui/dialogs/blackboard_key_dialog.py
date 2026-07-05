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

from typing import Dict, Optional
from yasmin_editor.qt_compat import QtWidgets


class BlackboardKeyDialog(QtWidgets.QDialog):
    """Dialog for creating and editing blackboard keys."""

    TYPE_OPTIONS = ["in", "out", "in/out"]
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
        key_data: Optional[Dict[str, str]] = None,
        parent=None,
        edit_mode: bool = False,
        readonly: bool = False,
    ) -> None:
        super().__init__(parent)
        self.readonly = readonly
        title = "Edit Blackboard Key" if key_data else "Add Blackboard Key"
        if self.readonly:
            title += " (Readonly)"
        self.setWindowTitle(title)
        self.edit_mode = edit_mode
        self.resize(480, 320)

        key_data = dict(key_data or {})

        layout = QtWidgets.QFormLayout(self)

        self.name_edit = QtWidgets.QLineEdit(key_data.get("name", ""))
        self.name_edit.setPlaceholderText("Enter key name")
        self.name_edit.setReadOnly(self.edit_mode or self.readonly)
        layout.addRow("Name:*", self.name_edit)

        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(self.TYPE_OPTIONS)
        key_type = key_data.get("key_type", "in")
        type_index = self.type_combo.findText(key_type)
        if type_index >= 0:
            self.type_combo.setCurrentIndex(type_index)
        self.type_combo.setEnabled(not self.edit_mode and not self.readonly)
        layout.addRow("Type:", self.type_combo)

        self.description_edit = QtWidgets.QTextEdit()
        self.description_edit.setMaximumHeight(80)
        self.description_edit.setPlainText(key_data.get("description", ""))
        self.description_edit.setReadOnly(self.readonly)
        layout.addRow(QtWidgets.QLabel("<b>Description:</b>"), self.description_edit)

        self.default_type_combo = QtWidgets.QComboBox()
        self.default_type_combo.addItem("No default", "")
        for option in self.VALUE_TYPE_OPTIONS[1:]:
            self.default_type_combo.addItem(option, option)
        default_type = str(key_data.get("default_type", "") or "")
        default_type_index = self.default_type_combo.findData(default_type)
        if default_type_index >= 0:
            self.default_type_combo.setCurrentIndex(default_type_index)
        else:
            self.default_type_combo.setCurrentIndex(0)
        self.default_type_combo.currentIndexChanged.connect(
            self._update_default_value_state
        )
        self.default_type_combo.setEnabled(
            not self.readonly and self.default_type_combo.isEnabled()
        )
        layout.addRow("Default Type:", self.default_type_combo)

        self.default_value_edit = QtWidgets.QLineEdit(
            str(key_data.get("default_value", "") or "")
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

        self.type_combo.currentTextChanged.connect(self._update_default_fields)
        self._update_default_fields(self.type_combo.currentText())

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

    def _update_default_fields(self, key_type: str) -> None:
        allow_defaults = key_type in ("in", "in/out")
        if not allow_defaults:
            self.default_type_combo.setCurrentIndex(0)
            self.default_value_edit.clear()

        self.default_type_combo.setEnabled(allow_defaults and not self.readonly)
        self._update_default_value_state()

    def _update_default_value_state(self) -> None:
        has_default_type = bool(self.default_type_combo.currentData())
        self.default_value_edit.setEnabled(
            self.default_type_combo.isEnabled() and has_default_type
        )

    def _accept_with_validation(self) -> None:
        if not self.name_edit.text().strip():
            QtWidgets.QMessageBox.warning(
                self, "Validation Error", "Key name is required!"
            )
            return
        self.accept()

    def get_key_data(self) -> Dict[str, str]:
        default_type = ""
        default_value = ""
        if self.type_combo.currentText() in ("in", "in/out"):
            default_type = str(self.default_type_combo.currentData() or "")
            if default_type:
                default_value = self.default_value_edit.text().strip()

        return {
            "name": self.name_edit.text().strip(),
            "key_type": self.type_combo.currentText(),
            "description": self.description_edit.toPlainText().strip(),
            "default_type": default_type,
            "default_value": default_value,
        }
