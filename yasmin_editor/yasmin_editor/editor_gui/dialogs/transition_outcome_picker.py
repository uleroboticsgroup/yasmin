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

from __future__ import annotations

from typing import List, Union
from collections.abc import Sequence
from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets


class _OutcomeListWidget(QtWidgets.QListWidget):
    """Outcome list with checkbox toggling for the current item."""

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_Space:
            item = self.currentItem()
            if item is not None and not item.isHidden():
                is_checked = item.checkState() == Qt.CheckState.Checked
                item.setCheckState(
                    Qt.CheckState.Unchecked if is_checked else Qt.CheckState.Checked
                )
                event.accept()
                return
        super().keyPressEvent(event)


class TransitionOutcomePickerDialog(QtWidgets.QDialog):
    """Searchable transition outcome picker with explicit checkbox selection."""

    def __init__(
        self,
        *,
        parent: Union[QtWidgets.QWidget, None],
        source_name: str,
        available_outcomes: Sequence[str],
    ) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle("Select Outcomes")
        self.setMinimumWidth(360)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)

        self.available_outcomes = [str(item) for item in available_outcomes]
        self._popup_offset = QtCore.QPoint(14, 14)

        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel(f"Connect outcomes from <b>{source_name}</b>")
        title.setTextFormat(Qt.TextFormat.RichText)
        layout.addWidget(title)

        self.search_edit = QtWidgets.QLineEdit(self)
        self.search_edit.setPlaceholderText("Filter outcomes…")
        layout.addWidget(self.search_edit)

        self.outcome_list = _OutcomeListWidget(self)
        self.outcome_list.setAlternatingRowColors(True)
        self.outcome_list.setSelectionMode(
            QtWidgets.QAbstractItemView.SelectionMode.SingleSelection
        )
        layout.addWidget(self.outcome_list)

        for outcome_name in self.available_outcomes:
            item = QtWidgets.QListWidgetItem(outcome_name)
            item.setData(Qt.ItemDataRole.UserRole, outcome_name)
            item.setFlags(
                item.flags()
                | Qt.ItemFlag.ItemIsUserCheckable
                | Qt.ItemFlag.ItemIsSelectable
            )
            item.setCheckState(Qt.CheckState.Unchecked)
            self.outcome_list.addItem(item)

        button_box = QtWidgets.QDialogButtonBox(self)
        self.accept_button = button_box.addButton(
            "Connect Selected", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole
        )
        button_box.addButton(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        layout.addWidget(button_box)

        self.search_edit.textChanged.connect(self._apply_filter)
        self.outcome_list.itemChanged.connect(self._update_accept_button_state)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        self._apply_filter("")
        self.search_edit.setFocus(Qt.FocusReason.ActiveWindowFocusReason)

    def showEvent(self, event) -> None:  # type: ignore[override]
        super().showEvent(event)
        self._move_near_cursor()
        self.search_edit.setFocus(Qt.FocusReason.ActiveWindowFocusReason)
        self.search_edit.selectAll()

    def _move_near_cursor(self) -> None:
        cursor_pos = QtGui.QCursor.pos() + self._popup_offset
        screen = self.screen()
        if screen is not None:
            available = screen.availableGeometry()
            x = min(cursor_pos.x(), available.right() - self.width())
            y = min(cursor_pos.y(), available.bottom() - self.height())
            cursor_pos = QtCore.QPoint(max(available.left(), x), max(available.top(), y))
        self.move(cursor_pos)

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        first_visible = None
        for index in range(self.outcome_list.count()):
            item = self.outcome_list.item(index)
            outcome_name = str(item.data(Qt.ItemDataRole.UserRole) or "")
            is_match = not needle or needle in outcome_name.lower()
            item.setHidden(not is_match)
            if is_match and first_visible is None:
                first_visible = item
        if first_visible is not None:
            self.outcome_list.setCurrentItem(first_visible)
        self._update_accept_button_state()

    def _update_accept_button_state(self) -> None:
        self.accept_button.setEnabled(bool(self.selected_outcomes()))

    def selected_outcomes(self) -> List[str]:
        selected: List[str] = []
        for index in range(self.outcome_list.count()):
            item = self.outcome_list.item(index)
            outcome_name = str(item.data(Qt.ItemDataRole.UserRole) or "")
            if outcome_name and item.checkState() == Qt.CheckState.Checked:
                selected.append(outcome_name)
        return selected
