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
from PyQt5.QtWidgets import (
    QLabel,
    QDialog,
    QFormLayout,
    QLineEdit,
    QComboBox,
    QDialogButtonBox,
    QTextEdit,
)
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSizePolicy

from yasmin_editor.plugins_manager.plugin_info import PluginInfo


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
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit State Properties" if edit_mode else "Add State")
        self.resize(500, 500)
        self.edit_mode: bool = edit_mode

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

        # Pre-select plugin if provided
        if plugin_info and self.type_combo.currentIndex() in [0, 1, 2]:
            for i in range(self.plugin_combo.count()):
                if self.plugin_combo.itemData(i) == plugin_info:
                    self.plugin_combo.setCurrentIndex(i)
                    break

        self.plugin_combo.currentIndexChanged.connect(self.update_outcome_list)
        self.type_combo.currentIndexChanged.connect(self.update_plugin_list)

        # Disable plugin selection in edit mode (can't change plugin type)
        if edit_mode:
            self.plugin_combo.setEnabled(False)

        # Outcomes field (for new state machines and concurrence)
        self.outcomes_label: QLabel = QLabel("Outcomes:")
        if plugin_info and hasattr(plugin_info, "outcomes"):
            outcomes_str: str = ", ".join(plugin_info.outcomes)
        elif outcomes:
            outcomes_str = ", ".join(outcomes)
        else:
            outcomes_str = ""

        self.outcomes_display: QLabel = QLabel(outcomes_str)
        self.outcomes_display.setStyleSheet(
            "background: #f0f0f0; border: 1px solid #ccc; padding: 4px;"
        )
        self.outcomes_display.setWordWrap(True)
        self.outcomes_display.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.outcomes_display.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        # Dynamically set min/max height based on number of outcomes
        num_outcomes: int = len(outcomes_str.split(",")) if outcomes_str else 0
        base_height: int = 24
        extra_height: int = min(3, num_outcomes) * 18  # up to 3 lines
        self.outcomes_display.setMinimumHeight(base_height + extra_height)
        self.outcomes_display.setMaximumHeight(base_height + max(3, num_outcomes) * 18)
        layout.addRow(self.outcomes_label, self.outcomes_display)

        # Remappings
        remappings_label: QLabel = QLabel("<b>Remappings (optional):</b>")

        self.remappings_edit: QTextEdit = QTextEdit()
        self.remappings_edit.setMaximumHeight(100)
        self.remappings_edit.setPlaceholderText(
            "old_key:new_key\nanother_key:another_value"
        )
        if remappings:
            remap_text: str = "\n".join([f"{k}:{v}" for k, v in remappings.items()])
            self.remappings_edit.setPlainText(remap_text)
        layout.addRow(remappings_label, self.remappings_edit)

        self.update_plugin_list()
        self.update_outcome_list()

        # Buttons
        buttons: QDialogButtonBox = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def update_outcome_list(self) -> None:
        current_type: int = self.type_combo.currentIndex()
        plugin_info: Optional[PluginInfo] = None
        if current_type in [0, 1, 2]:
            plugin_info = self.plugin_combo.currentData()

        if plugin_info and hasattr(plugin_info, "outcomes"):
            outcomes_str: str = ", ".join(plugin_info.outcomes)
        else:
            outcomes_str = ""
        self.outcomes_display.setText(outcomes_str)

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

    def get_state_data(
        self,
    ) -> Tuple[
        str,
        Optional[PluginInfo],
        List[str],
        Dict[str, str],
    ]:
        """Returns: (name, plugin_info, outcomes, remappings)"""
        name: str = self.name_edit.text().strip()
        plugin: Optional[PluginInfo] = self.plugin_combo.currentData()

        remappings: Dict[str, str] = {}
        remap_text: str = self.remappings_edit.toPlainText().strip()
        for line in remap_text.split("\n"):
            line = line.strip()
            if ":" in line:
                key: str
                value: str
                key, value = line.split(":", 1)
                remappings[key.strip()] = value.strip()

        outcomes: str = self.outcomes_display.text().strip()
        outcomes_list: List[str] = [o.strip() for o in outcomes.split(",") if o.strip()]

        return name, plugin, outcomes_list, remappings
