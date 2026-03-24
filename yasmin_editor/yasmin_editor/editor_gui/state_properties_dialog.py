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

import os
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QComboBox, QDialog, QDialogButtonBox, QFormLayout,
                             QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QPushButton, QSizePolicy, QTableWidget,
                             QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)
from yasmin_plugins_manager.plugin_info import PluginInfo


class StatePropertiesDialog(QDialog):
    """Dialog for setting state properties."""

    def __init__(
        self,
        state_name: str = "",
        plugin_info: Optional[PluginInfo] = None,
        available_plugins: Optional[List[PluginInfo]] = None,
        remappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
        edit_mode: bool = False,
        parent: Optional[QDialog] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit State Properties" if edit_mode else "Add State")
        self.resize(600, 700)
        self.edit_mode: bool = edit_mode
        self._base_description: str = description
        self._fallback_outcomes: List[str] = outcomes or []

        layout: QFormLayout = QFormLayout(self)

        # State name
        self.name_edit: QLineEdit = QLineEdit(state_name)
        self.name_edit.setPlaceholderText("Enter state name (required)")
        layout.addRow("Name:*", self.name_edit)

        # State type
        self.type_combo: QComboBox = QComboBox()
        self.type_combo.addItem("Python State")
        self.type_combo.addItem("C++ State")
        self.type_combo.addItem("XML File")

        # Set initial type based on parameters
        if plugin_info:
            if plugin_info.plugin_type == "python":
                self.type_combo.setCurrentIndex(0)
            elif plugin_info.plugin_type == "cpp":
                self.type_combo.setCurrentIndex(1)
            elif plugin_info.plugin_type == "xml":
                self.type_combo.setCurrentIndex(2)

        # Disable type change in edit mode
        if edit_mode:
            self.type_combo.setEnabled(False)

        layout.addRow("Type:", self.type_combo)

        # Plugin selection (for Python/C++/XML)
        self.plugin_label: QLabel = QLabel("Plugin:*")
        self.plugin_combo: QComboBox = QComboBox()
        self.available_plugins: List[PluginInfo] = available_plugins or []

        layout.addRow(self.plugin_label, self.plugin_combo)

        self.plugin_combo.currentIndexChanged.connect(self.update_description)
        self.type_combo.currentIndexChanged.connect(self.update_plugin_list)

        # Disable plugin selection in edit mode (can't change plugin type)
        if edit_mode:
            self.plugin_combo.setEnabled(False)

        # State description (read-only — loaded from plugin metadata)
        desc_label: QLabel = QLabel("<b>Description:</b>")
        self.description_edit: QTextEdit = QTextEdit()
        self.description_edit.setMinimumHeight(240)
        self.description_edit.setMaximumHeight(320)
        self.description_edit.setReadOnly(True)
        self.description_edit.setStyleSheet(
            "background: #f0f0f0; border: 1px solid #ccc; color: #333;"
        )
        layout.addRow(desc_label, self.description_edit)

        # Remappings
        remappings_label: QLabel = QLabel("<b>Remappings (optional):</b>")
        remappings_widget: QWidget = QWidget()
        remappings_layout: QVBoxLayout = QVBoxLayout(remappings_widget)
        remappings_layout.setContentsMargins(0, 0, 0, 0)

        self.remappings_table: QTableWidget = QTableWidget(0, 2)
        self.remappings_table.setHorizontalHeaderLabels(["Old Key", "New Key"])
        self.remappings_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.remappings_table.setMinimumHeight(80)
        self.remappings_table.setMaximumHeight(150)
        remappings_layout.addWidget(self.remappings_table)

        remap_btn_layout: QHBoxLayout = QHBoxLayout()
        add_remap_btn: QPushButton = QPushButton("Add Row")
        add_remap_btn.clicked.connect(self.add_remapping_row)
        remap_btn_layout.addWidget(add_remap_btn)
        remove_remap_btn: QPushButton = QPushButton("Remove Row")
        remove_remap_btn.clicked.connect(self.remove_remapping_row)
        remap_btn_layout.addWidget(remove_remap_btn)
        remap_btn_layout.addStretch()
        remappings_layout.addLayout(remap_btn_layout)

        if remappings:
            for old_key, new_key in remappings.items():
                self._add_remapping_row_with_data(old_key, new_key)

        layout.addRow(remappings_label, remappings_widget)

        self.update_plugin_list()

        # Pre-select plugin if provided
        if plugin_info and self.type_combo.currentIndex() in [0, 1, 2]:
            for i in range(self.plugin_combo.count()):
                if self.plugin_combo.itemData(i) == plugin_info:
                    self.plugin_combo.setCurrentIndex(i)
                    break

        self.update_description()

        if not self.plugin_combo.currentData():
            self.description_edit.setPlainText(
                self._build_description_text(
                    None,
                    fallback_description=self._base_description,
                    fallback_outcomes=self._fallback_outcomes,
                )
            )

        # Buttons
        buttons: QDialogButtonBox = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _format_key_line(self, key_info: Dict[str, str], is_input: bool) -> str:
        key_name = str(key_info.get("name", "")).strip()
        key_desc = str(key_info.get("description", "")).strip()
        key_type = str(
            key_info.get("type", key_info.get("default_value_type", ""))
        ).strip()

        line = key_name if key_name else "(unnamed)"

        if key_desc:
            line += f": {key_desc}"

        if is_input and key_info.get("has_default"):
            default_value = str(key_info.get("default_value", "")).strip()
            line += f" Default: {default_value}"

        if key_type:
            line += f" ({key_type})"

        return line

    def _build_description_text(
        self,
        plugin_info: Optional[PluginInfo],
        fallback_description: str = "",
        fallback_outcomes: Optional[List[str]] = None,
    ) -> str:
        if plugin_info:
            base_description = str(getattr(plugin_info, "description", "") or "").strip()
            input_keys = list(getattr(plugin_info, "input_keys", []) or [])
            output_keys = list(getattr(plugin_info, "output_keys", []) or [])
            outcomes = list(getattr(plugin_info, "outcomes", []) or [])
            outcome_descriptions = dict(
                getattr(plugin_info, "outcome_descriptions", {}) or {}
            )
        else:
            base_description = fallback_description.strip()
            input_keys = []
            output_keys = []
            outcomes = list(fallback_outcomes or [])
            outcome_descriptions = {}

        sections: List[str] = []
        if base_description:
            sections.append(base_description)

        if outcomes:
            if sections:
                sections.append("")
            sections.append("Outcomes:")
            for outcome in outcomes:
                line = f" - {outcome}"
                desc = outcome_descriptions.get(outcome)
                if desc:
                    line += f": {desc}"
                sections.append(line)

        if input_keys:
            if sections:
                sections.append("")
            lines = ["Input Keys:"]
            for key_info in input_keys:
                lines.append(" - " + self._format_key_line(key_info, True))
            sections.append("\n".join(lines))

        if output_keys:
            if sections:
                sections.append("")
            lines = ["Output Keys:"]
            for key_info in output_keys:
                lines.append(" - " + self._format_key_line(key_info, False))
            sections.append("\n".join(lines))

        return "\n".join(sections).strip()

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
        """Add a row to the remappings table pre-filled with data."""
        row = self.remappings_table.rowCount()
        self.remappings_table.insertRow(row)
        self.remappings_table.setItem(row, 0, QTableWidgetItem(old_key))
        self.remappings_table.setItem(row, 1, QTableWidgetItem(new_key))

    def update_description(self) -> None:
        plugin_info: Optional[PluginInfo] = self.plugin_combo.currentData()
        description_text = self._build_description_text(
            plugin_info,
            fallback_description=self._base_description,
            fallback_outcomes=self._fallback_outcomes,
        )
        self.description_edit.setPlainText(description_text)

    def update_plugin_list(self) -> None:
        self.plugin_combo.clear()

        current_type: int = self.type_combo.currentIndex()
        if current_type == 0:  # Python
            for plugin in self.available_plugins:
                if plugin.plugin_type == "python":
                    self.plugin_combo.addItem(
                        f"{plugin.module}.{plugin.class_name}", plugin
                    )
        elif current_type == 1:  # C++
            for plugin in self.available_plugins:
                if plugin.plugin_type == "cpp":
                    self.plugin_combo.addItem(plugin.class_name, plugin)
        elif current_type == 2:  # State Machine (XML)
            for plugin in self.available_plugins:
                if plugin.plugin_type == "xml":
                    filename: str = os.path.basename(plugin.file_name)
                    display_name: str = (
                        f"{plugin.package_name}::{filename}"
                        if plugin.package_name
                        else filename
                    )
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
    ]:
        """Returns: (name, plugin_info, outcomes, remappings, description, defaults)"""
        name: str = self.name_edit.text().strip()
        plugin: Optional[PluginInfo] = self.plugin_combo.currentData()

        remappings: Dict[str, str] = {}
        for row in range(self.remappings_table.rowCount()):
            old_item = self.remappings_table.item(row, 0)
            new_item = self.remappings_table.item(row, 1)
            old_key = old_item.text().strip() if old_item else ""
            new_key = new_item.text().strip() if new_item else ""
            if old_key and new_key:
                remappings[old_key] = new_key

        if plugin and hasattr(plugin, "outcomes"):
            outcomes_list: List[str] = list(plugin.outcomes)
        else:
            outcomes_list = list(self._fallback_outcomes)

        description: str = self.description_edit.toPlainText().strip()

        return name, plugin, outcomes_list, remappings, description, []
