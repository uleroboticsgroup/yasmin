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
from PyQt5.QtWidgets import (
    QLabel,
    QDialog,
    QFormLayout,
    QLineEdit,
    QComboBox,
    QDialogButtonBox,
    QTextEdit,
)

from yasmin_editor.plugins_manager.plugin_info import PluginInfo


class StatePropertiesDialog(QDialog):
    """Dialog for setting state properties."""

    def __init__(
        self,
        state_name: str = "",
        plugin_info: PluginInfo = None,
        available_plugins: List[PluginInfo] = None,
        remappings: Dict[str, str] = None,
        outcomes: List[str] = None,
        edit_mode: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Edit State Properties" if edit_mode else "Add State")
        self.resize(500, 500)
        self.edit_mode = edit_mode

        layout = QFormLayout(self)

        # State name
        self.name_edit = QLineEdit(state_name)
        self.name_edit.setPlaceholderText("Enter state name (required)")
        layout.addRow("Name:*", self.name_edit)

        # State type
        self.type_combo = QComboBox()
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

        # Plugin selection (for Python/C++/XML) - CREATE LABEL BEFORE USING IT
        self.plugin_label = QLabel("Plugin:*")
        self.plugin_combo = QComboBox()
        self.available_plugins = available_plugins or []

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
        self.outcomes_label = QLabel("Outcomes (space-separated):*")
        self.outcomes_edit = QLineEdit()
        self.outcomes_edit.setPlaceholderText("e.g., outcome1 outcome2 outcome3")

        # Outcomes are read-only
        self.outcomes_edit.setEnabled(False)
        self.outcomes_label.setText("Outcomes :")
        if plugin_info and hasattr(plugin_info, "outcomes"):
            self.outcomes_edit.setText(" ".join(plugin_info.outcomes))
        elif outcomes:
            self.outcomes_edit.setText(" ".join(outcomes))
        layout.addRow(self.outcomes_label, self.outcomes_edit)

        # Remappings
        remappings_label = QLabel("<b>Remappings (optional):</b>")

        self.remappings_edit = QTextEdit()
        self.remappings_edit.setMaximumHeight(100)
        self.remappings_edit.setPlaceholderText(
            "old_key:new_key\nanother_key:another_value"
        )
        if remappings:
            remap_text = "\n".join([f"{k}:{v}" for k, v in remappings.items()])
            self.remappings_edit.setPlainText(remap_text)
        layout.addRow(remappings_label, self.remappings_edit)

        self.update_plugin_list()
        self.update_outcome_list()

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def update_outcome_list(self):
        current_type = self.type_combo.currentIndex()
        plugin_info: PluginInfo = None
        if current_type in [0, 1, 2]:
            plugin_info = self.plugin_combo.currentData()

        if plugin_info and hasattr(plugin_info, "outcomes"):
            self.outcomes_edit.setText(" ".join(plugin_info.outcomes))
        else:
            self.outcomes_edit.setText(" ")

    def update_plugin_list(self):
        self.plugin_combo.clear()

        current_type = self.type_combo.currentIndex()
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
                    import os

                    filename = os.path.basename(plugin.file_path)
                    display_name = (
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
        """
        Returns: (name, plugin_info, outcomes, remappings)
        """
        name = self.name_edit.text().strip()
        plugin = self.plugin_combo.currentData()

        # Parse remappings
        remappings = {}
        remap_text = self.remappings_edit.toPlainText().strip()
        if remap_text:
            for line in remap_text.split("\n"):
                line = line.strip()
                if ":" in line:
                    key, value = line.split(":", 1)
                    remappings[key.strip()] = value.strip()

        # Parse outcomes for new SM/Concurrence
        outcomes = self.outcomes_edit.text().strip()

        return (
            name,
            plugin,
            outcomes,
            remappings,
        )
