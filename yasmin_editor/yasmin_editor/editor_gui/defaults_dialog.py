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

from typing import Dict, List, Optional

from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
)


class DefaultsDialog(QDialog):
    """Dialog for editing a list of default blackboard values."""

    def __init__(
        self,
        defaults: Optional[List[Dict[str, str]]] = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Default Values")
        self.resize(580, 360)

        layout: QVBoxLayout = QVBoxLayout(self)
        layout.addWidget(QLabel("<b>Default Values:</b>"))

        self.table: QTableWidget = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Key", "Value", "Type"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)

        btn_layout: QHBoxLayout = QHBoxLayout()
        add_btn: QPushButton = QPushButton("Add Row")
        add_btn.clicked.connect(self._add_row)
        btn_layout.addWidget(add_btn)
        remove_btn: QPushButton = QPushButton("Remove Row")
        remove_btn.clicked.connect(self._remove_row)
        btn_layout.addWidget(remove_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        buttons: QDialogButtonBox = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        for row_data in defaults or []:
            self._add_row_with_data(row_data)

    def _add_row(self) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        type_combo = QComboBox()
        type_combo.addItems(["str", "int", "float", "bool"])
        self.table.setCellWidget(row, 2, type_combo)

    def _remove_row(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)

    def _add_row_with_data(self, data: Dict[str, str]) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(data.get("key", "")))
        self.table.setItem(row, 1, QTableWidgetItem(data.get("value", "")))
        type_combo = QComboBox()
        type_combo.addItems(["str", "int", "float", "bool"])
        type_str = data.get("type", "str")
        for i in range(type_combo.count()):
            if type_combo.itemText(i) in type_str:
                type_combo.setCurrentIndex(i)
                break
        self.table.setCellWidget(row, 2, type_combo)

    def get_defaults(self) -> List[Dict[str, str]]:
        """Return the current list of defaults from the table."""
        defaults = []
        for row in range(self.table.rowCount()):
            key_item = self.table.item(row, 0)
            value_item = self.table.item(row, 1)
            type_widget = self.table.cellWidget(row, 2)
            desc_item = self.table.item(row, 3)
            key = key_item.text().strip() if key_item else ""
            value = value_item.text().strip() if value_item else ""
            type_str = type_widget.currentText() if type_widget else "str"
            if key:
                defaults.append({"key": key, "value": value, "type": type_str})
        return defaults
