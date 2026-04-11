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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from yasmin_plugins_manager.plugin_info import PluginInfo

from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
    ParameterOverwriteDialog,
)
from yasmin_editor.editor_gui.state_properties_logic import (
    build_description_text,
    collect_parameter_overwrites,
    collect_remappings,
    declared_state_parameters,
    plugin_entries_for_type,
    resolve_outcomes,
)


class StatePropertiesDialog(QDialog):
    """Dialog for setting state properties."""

    def __init__(
        self,
        state_name: str = "",
        plugin_info: Optional[PluginInfo] = None,
        available_plugins: Optional[List[PluginInfo]] = None,
        remappings: Optional[Dict[str, str]] = None,
        parameter_overwrites: Optional[List[Dict[str, str]]] = None,
        declared_parent_parameters: Optional[List[Dict[str, str]]] = None,
        outcomes: Optional[List[str]] = None,
        edit_mode: bool = False,
        parent: Optional[QDialog] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
        fallback_input_keys: Optional[List[Dict[str, str]]] = None,
        fallback_output_keys: Optional[List[Dict[str, str]]] = None,
        fallback_parameters: Optional[List[Dict[str, str]]] = None,
        container_kind: Optional[str] = None,
        readonly: bool = False,
        enable_parameter_overwrites: bool = True,
    ) -> None:
        super().__init__(parent)
        self.readonly: bool = readonly
        title = "Edit State Properties" if edit_mode else "Add State"
        if self.readonly:
            title += " (Readonly)"
        self.setWindowTitle(title)
        self.resize(700, 820)
        self.edit_mode: bool = edit_mode
        self._base_description: str = description
        self._fallback_outcomes: List[str] = outcomes or []
        self._fallback_input_keys: List[Dict[str, str]] = fallback_input_keys or []
        self._fallback_output_keys: List[Dict[str, str]] = fallback_output_keys or []
        self._fallback_parameters: List[Dict[str, str]] = fallback_parameters or []
        self._container_kind: Optional[str] = container_kind
        self._declared_parent_parameters: Dict[str, Dict[str, str]] = {
            str(item.get("name", "") or "").strip(): dict(item)
            for item in (declared_parent_parameters or [])
            if str(item.get("name", "") or "").strip()
        }
        self._enable_parameter_overwrites = enable_parameter_overwrites

        layout: QFormLayout = QFormLayout(self)
        self._layout = layout

        self.name_edit: QLineEdit = QLineEdit(state_name)
        self.name_edit.setPlaceholderText("Enter state name (required)")
        self.name_edit.setReadOnly(self.readonly)
        layout.addRow("Name:*", self.name_edit)

        self.type_combo: QComboBox = QComboBox()
        self.type_combo.addItem("Python State")
        self.type_combo.addItem("C++ State")
        self.type_combo.addItem("XML File")

        if self._container_kind:
            self.type_combo.clear()
            self.type_combo.addItem(self._container_kind)
            self.type_combo.setEnabled(False)
        elif plugin_info:
            if plugin_info.plugin_type == "python":
                self.type_combo.setCurrentIndex(0)
            elif plugin_info.plugin_type == "cpp":
                self.type_combo.setCurrentIndex(1)
            elif plugin_info.plugin_type == "xml":
                self.type_combo.setCurrentIndex(2)

        if edit_mode or self.readonly:
            self.type_combo.setEnabled(False)

        layout.addRow("Type:", self.type_combo)

        self.plugin_label: QLabel = QLabel("Plugin:*")
        self.plugin_combo: QComboBox = QComboBox()
        self.available_plugins: List[PluginInfo] = available_plugins or []
        layout.addRow(self.plugin_label, self.plugin_combo)

        self.plugin_combo.currentIndexChanged.connect(self.update_description)
        self.type_combo.currentIndexChanged.connect(self.update_plugin_list)

        if edit_mode or self.readonly:
            self.plugin_combo.setEnabled(False)

        if self._container_kind:
            self.plugin_label.hide()
            self.plugin_combo.hide()

        desc_label: QLabel = QLabel("<b>Description:</b>")
        self.description_edit: QTextEdit = QTextEdit()
        self.description_edit.setMinimumHeight(240)
        self.description_edit.setMaximumHeight(320)
        self.description_edit.setReadOnly(True)
        self.description_edit.setProperty("viewerText", True)
        layout.addRow(desc_label, self.description_edit)

        self.parameter_table = QTableWidget(0, 4)
        self.parameter_table.setHorizontalHeaderLabels(
            ["Name", "Overrides", "Type", "Default"]
        )
        self.parameter_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.parameter_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.parameter_table.setSelectionMode(QTableWidget.SingleSelection)
        self.parameter_table.setMinimumHeight(100)
        self.parameter_table.setMaximumHeight(180)
        self.parameter_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.parameter_table.setEnabled(
            self._enable_parameter_overwrites and not self.readonly
        )
        self.parameter_table.itemDoubleClicked.connect(self.edit_parameter_overwrite_row)

        if self._enable_parameter_overwrites:
            params_label = QLabel("<b>Params:</b>")
            params_widget = QWidget()
            params_layout = QVBoxLayout(params_widget)
            params_layout.setContentsMargins(0, 0, 0, 0)
            params_layout.addWidget(self.parameter_table)

            param_btn_layout = QHBoxLayout()
            add_param_btn = QPushButton("Add Param Overwrite")
            add_param_btn.setEnabled(not self.readonly)
            add_param_btn.clicked.connect(self.add_parameter_overwrite_row)
            param_btn_layout.addWidget(add_param_btn)

            remove_param_btn = QPushButton("Remove Param Overwrite")
            remove_param_btn.setEnabled(not self.readonly)
            remove_param_btn.clicked.connect(self.remove_parameter_overwrite_row)
            param_btn_layout.addWidget(remove_param_btn)
            param_btn_layout.addStretch()
            params_layout.addLayout(param_btn_layout)
            layout.addRow(params_label, params_widget)

            for parameter_data in parameter_overwrites or []:
                self._add_parameter_row_with_data(parameter_data)

        remappings_label: QLabel = QLabel("<b>Remappings:</b>")
        remappings_widget: QWidget = QWidget()
        remappings_layout: QVBoxLayout = QVBoxLayout(remappings_widget)
        remappings_layout.setContentsMargins(0, 0, 0, 0)

        self.remappings_table: QTableWidget = QTableWidget(0, 2)
        self.remappings_table.setHorizontalHeaderLabels(["Old Key", "New Key"])
        self.remappings_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.remappings_table.setMinimumHeight(80)
        self.remappings_table.setMaximumHeight(150)
        self.remappings_table.setEnabled(not self.readonly)
        remappings_layout.addWidget(self.remappings_table)

        remap_btn_layout: QHBoxLayout = QHBoxLayout()
        add_remap_btn: QPushButton = QPushButton("Add Row")
        add_remap_btn.setEnabled(not self.readonly)
        add_remap_btn.clicked.connect(self.add_remapping_row)
        remap_btn_layout.addWidget(add_remap_btn)
        remove_remap_btn: QPushButton = QPushButton("Remove Row")
        remove_remap_btn.setEnabled(not self.readonly)
        remove_remap_btn.clicked.connect(self.remove_remapping_row)
        remap_btn_layout.addWidget(remove_remap_btn)
        remap_btn_layout.addStretch()
        remappings_layout.addLayout(remap_btn_layout)

        if remappings:
            for old_key, new_key in remappings.items():
                self._add_remapping_row_with_data(old_key, new_key)

        layout.addRow(remappings_label, remappings_widget)

        self.update_plugin_list()

        if plugin_info and not self._container_kind:
            for i in range(self.plugin_combo.count()):
                if self.plugin_combo.itemData(i) == plugin_info:
                    self.plugin_combo.setCurrentIndex(i)
                    break

        self.update_description()

        if self._container_kind or not self.plugin_combo.currentData():
            self.description_edit.setPlainText(
                build_description_text(
                    None,
                    fallback_description=self._base_description,
                    fallback_outcomes=self._fallback_outcomes,
                    fallback_parameters=self._fallback_parameters,
                    fallback_input_keys=self._fallback_input_keys,
                    fallback_output_keys=self._fallback_output_keys,
                )
            )

        buttons: QDialogButtonBox = QDialogButtonBox(
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

    def _declared_state_parameters(self) -> List[Dict[str, str]]:
        plugin_info: Optional[PluginInfo] = (
            None if self._container_kind else self.plugin_combo.currentData()
        )
        return declared_state_parameters(plugin_info, self._fallback_parameters)

    def add_parameter_overwrite_row(self) -> None:
        dialog = ParameterOverwriteDialog(
            declared_parameters=self._declared_state_parameters(),
            parent=self,
        )
        if dialog.exec_():
            self._add_parameter_row_with_data(dialog.get_parameter_data())

    def edit_parameter_overwrite_row(self, *_args) -> None:
        if self.readonly or not self._enable_parameter_overwrites:
            return
        row = self.parameter_table.currentRow()
        if row < 0:
            return
        current_data = self._parameter_row_data(row)
        dialog = ParameterOverwriteDialog(
            declared_parameters=self._declared_state_parameters(),
            param_data=current_data,
            parent=self,
        )
        if dialog.exec_():
            self._set_parameter_row_data(row, dialog.get_parameter_data())

    def remove_parameter_overwrite_row(self) -> None:
        row = self.parameter_table.currentRow()
        if row >= 0:
            self.parameter_table.removeRow(row)

    def _parameter_row_data(self, row: int) -> Dict[str, str]:
        name_item = self.parameter_table.item(row, 0)
        child_item = self.parameter_table.item(row, 1)
        type_item = self.parameter_table.item(row, 2)
        default_item = self.parameter_table.item(row, 3)
        return {
            "name": name_item.text().strip() if name_item else "",
            "child_parameter": child_item.data(Qt.UserRole) if child_item else "",
            "description": name_item.data(Qt.UserRole) if name_item else "",
            "default_type": type_item.text().strip() if type_item else "",
            "default_value": default_item.text().strip() if default_item else "",
        }

    def _set_parameter_row_data(self, row: int, parameter_data: Dict[str, str]) -> None:
        name = str(parameter_data.get("name", "") or "").strip()
        child_parameter = str(parameter_data.get("child_parameter", "") or "").strip()
        description = str(parameter_data.get("description", "") or "").strip()
        default_type = str(parameter_data.get("default_type", "") or "").strip()
        default_value = str(parameter_data.get("default_value", "") or "").strip()

        name_item = QTableWidgetItem(name)
        name_item.setData(Qt.UserRole, description)
        if description:
            name_item.setToolTip(description)

        child_item = QTableWidgetItem(child_parameter)
        child_item.setData(Qt.UserRole, child_parameter)

        type_item = QTableWidgetItem(default_type)
        default_item = QTableWidgetItem(default_value)

        self.parameter_table.setItem(row, 0, name_item)
        self.parameter_table.setItem(row, 1, child_item)
        self.parameter_table.setItem(row, 2, type_item)
        self.parameter_table.setItem(row, 3, default_item)

    def _add_parameter_row_with_data(self, parameter_data: Dict[str, str]) -> None:
        row = self.parameter_table.rowCount()
        self.parameter_table.insertRow(row)
        self._set_parameter_row_data(row, parameter_data)

    def get_parameter_overwrites(self) -> List[Dict[str, str]]:
        return collect_parameter_overwrites(
            self._parameter_row_data(row)
            for row in range(self.parameter_table.rowCount())
        )

    def add_remapping_row(self) -> None:
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)

    def remove_remapping_row(self) -> None:
        row = self.remappings_table.currentRow()
        if row >= 0:
            self.remappings_table.removeRow(row)

    def _add_remapping_row_with_data(self, old_key: str, new_key: str) -> None:
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)
        self.remappings_table.setItem(row, 0, QTableWidgetItem(old_key))
        self.remappings_table.setItem(row, 1, QTableWidgetItem(new_key))

    def update_description(self) -> None:
        plugin_info: Optional[PluginInfo] = (
            None if self._container_kind else self.plugin_combo.currentData()
        )
        description_text = build_description_text(
            plugin_info,
            fallback_description=self._base_description,
            fallback_outcomes=self._fallback_outcomes,
            fallback_parameters=self._fallback_parameters,
            fallback_input_keys=self._fallback_input_keys,
            fallback_output_keys=self._fallback_output_keys,
        )
        self.description_edit.setPlainText(description_text)

    def update_plugin_list(self) -> None:
        """Refresh the plugin combo from the selected state type."""

        self.plugin_combo.clear()
        if self._container_kind:
            self.update_description()
            return

        plugin_types = {0: "python", 1: "cpp", 2: "xml"}
        current_type = plugin_types.get(self.type_combo.currentIndex())
        if current_type is not None:
            for display_name, plugin in plugin_entries_for_type(
                self.available_plugins, current_type
            ):
                self.plugin_combo.addItem(display_name, plugin)

        self.update_description()

    def get_state_data(
        self,
    ) -> Tuple[
        str,
        Optional[PluginInfo],
        List[str],
        Dict[str, str],
        str,
        List[Dict[str, str]],
        List[Dict[str, str]],
    ]:
        name: str = self.name_edit.text().strip()
        plugin: Optional[PluginInfo] = (
            None if self._container_kind else self.plugin_combo.currentData()
        )

        remapping_rows = []
        for row in range(self.remappings_table.rowCount()):
            old_item = self.remappings_table.item(row, 0)
            new_item = self.remappings_table.item(row, 1)
            remapping_rows.append(
                (
                    old_item.text() if old_item else "",
                    new_item.text() if new_item else "",
                )
            )
        remappings = collect_remappings(remapping_rows)

        outcomes_list = resolve_outcomes(plugin, self._fallback_outcomes)

        description: str = self.description_edit.toPlainText().strip()
        return (
            name,
            plugin,
            outcomes_list,
            remappings,
            description,
            [],
            self.get_parameter_overwrites(),
        )
