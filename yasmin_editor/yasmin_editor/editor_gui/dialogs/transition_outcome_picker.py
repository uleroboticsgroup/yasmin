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

"""Compact searchable outcome picker used when creating transitions."""

from __future__ import annotations

from collections.abc import Sequence

from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtGui import QCursor, QKeyEvent
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)


class _OutcomeListWidget(QListWidget):
    """Outcome list with checkbox toggling for the current item."""

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key_Space:
            item = self.currentItem()
            if item is not None and not item.isHidden():
                is_checked = item.checkState() == Qt.Checked
                item.setCheckState(Qt.Unchecked if is_checked else Qt.Checked)
                event.accept()
                return
        super().keyPressEvent(event)


class TransitionOutcomePickerDialog(QDialog):
    """Searchable transition outcome picker with explicit checkbox selection."""

    def __init__(
        self,
        *,
        parent: QWidget | None,
        source_name: str,
        available_outcomes: Sequence[str],
    ) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle("Select Outcomes")
        self.setMinimumWidth(360)
        self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)

        self.available_outcomes = [str(item) for item in available_outcomes]
        self._popup_offset = QPoint(14, 14)

        layout = QVBoxLayout(self)
        title = QLabel(f"Connect outcomes from <b>{source_name}</b>")
        title.setTextFormat(Qt.RichText)
        layout.addWidget(title)

        self.search_edit = QLineEdit(self)
        self.search_edit.setPlaceholderText("Filter outcomes…")
        layout.addWidget(self.search_edit)

        self.outcome_list = _OutcomeListWidget(self)
        self.outcome_list.setAlternatingRowColors(True)
        self.outcome_list.setSelectionMode(QListWidget.SingleSelection)
        layout.addWidget(self.outcome_list)

        for outcome_name in self.available_outcomes:
            item = QListWidgetItem(outcome_name)
            item.setData(Qt.UserRole, outcome_name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsSelectable)
            item.setCheckState(Qt.Unchecked)
            self.outcome_list.addItem(item)

        button_box = QDialogButtonBox(self)
        self.accept_button = button_box.addButton(
            "Connect Selected", QDialogButtonBox.AcceptRole
        )
        button_box.addButton(QDialogButtonBox.Cancel)
        layout.addWidget(button_box)

        self.search_edit.textChanged.connect(self._apply_filter)
        self.outcome_list.itemChanged.connect(self._update_accept_button_state)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        self._apply_filter("")
        self.search_edit.setFocus(Qt.ActiveWindowFocusReason)

    def showEvent(self, event) -> None:  # type: ignore[override]
        super().showEvent(event)
        self._move_near_cursor()
        self.search_edit.setFocus(Qt.ActiveWindowFocusReason)
        self.search_edit.selectAll()

    def _move_near_cursor(self) -> None:
        cursor_pos = QCursor.pos() + self._popup_offset
        screen = self.screen()
        if screen is not None:
            available = screen.availableGeometry()
            x = min(cursor_pos.x(), available.right() - self.width())
            y = min(cursor_pos.y(), available.bottom() - self.height())
            cursor_pos = QPoint(max(available.left(), x), max(available.top(), y))
        self.move(cursor_pos)

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        first_visible = None
        for index in range(self.outcome_list.count()):
            item = self.outcome_list.item(index)
            outcome_name = str(item.data(Qt.UserRole) or "")
            is_match = not needle or needle in outcome_name.lower()
            item.setHidden(not is_match)
            if is_match and first_visible is None:
                first_visible = item
        if first_visible is not None:
            self.outcome_list.setCurrentItem(first_visible)
        self._update_accept_button_state()

    def _update_accept_button_state(self) -> None:
        self.accept_button.setEnabled(bool(self.selected_outcomes()))

    def selected_outcomes(self) -> list[str]:
        selected: list[str] = []
        for index in range(self.outcome_list.count()):
            item = self.outcome_list.item(index)
            outcome_name = str(item.data(Qt.UserRole) or "")
            if outcome_name and item.checkState() == Qt.Checked:
                selected.append(outcome_name)
        return selected
