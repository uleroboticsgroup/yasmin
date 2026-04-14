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

# GUI tests for the transition outcome picker dialog.
#
# The picker supports filtering and keyboard interaction, so these tests verify
# both behaviors on the real QListWidget-based dialog.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest

from yasmin_editor.editor_gui.dialogs.transition_outcome_picker import (
    TransitionOutcomePickerDialog,
)


def _visible_outcome_names(dialog):
    # Collect the currently visible outcome rows after the search filter has
    # been applied.
    names = []
    for index in range(dialog.outcome_list.count()):
        item = dialog.outcome_list.item(index)
        if not item.isHidden():
            names.append(item.text())
    return names


def test_transition_outcome_picker_filters_and_selects_with_keyboard(qapp):
    # Filter the list, select the remaining entry with the keyboard, and accept
    # the dialog through the regular confirm button.
    dialog = TransitionOutcomePickerDialog(
        parent=None,
        source_name="worker",
        available_outcomes=["done", "retry", "failed"],
    )
    dialog.show()
    qapp.processEvents()

    dialog.search_edit.setFocus()
    QTest.keyClicks(dialog.search_edit, "ret")
    qapp.processEvents()

    assert _visible_outcome_names(dialog) == ["retry"]
    assert not dialog.accept_button.isEnabled()

    dialog.outcome_list.setFocus()
    QTest.keyClick(dialog.outcome_list, Qt.Key_Space)
    qapp.processEvents()

    assert dialog.accept_button.isEnabled()
    assert dialog.selected_outcomes() == ["retry"]

    QTest.mouseClick(dialog.accept_button, Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
    dialog.close()


def test_transition_outcome_picker_preserves_multiple_checked_outcomes_under_filter(qapp):
    # Already selected outcomes must remain selected even when the visible list
    # is narrowed by the search filter.
    dialog = TransitionOutcomePickerDialog(
        parent=None,
        source_name="worker",
        available_outcomes=["done", "retry", "failed"],
    )
    dialog.show()
    qapp.processEvents()

    dialog.outcome_list.setCurrentRow(0)
    QTest.keyClick(dialog.outcome_list, Qt.Key_Space)
    dialog.outcome_list.setCurrentRow(2)
    QTest.keyClick(dialog.outcome_list, Qt.Key_Space)
    qapp.processEvents()

    assert dialog.selected_outcomes() == ["done", "failed"]

    dialog.search_edit.setFocus()
    QTest.keyClicks(dialog.search_edit, "fa")
    qapp.processEvents()

    assert _visible_outcome_names(dialog) == ["failed"]
    assert dialog.selected_outcomes() == ["done", "failed"]

    dialog.close()
