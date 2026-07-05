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

import pytest

pytest.importorskip("yasmin_editor.qt_compat")
pytest.importorskip("yasmin_editor.qt_compat")
pytest.importorskip("yasmin_editor.qt_compat")

from yasmin_editor.qt_compat import Qt, QtWidgets
from yasmin_editor.qt_compat import QtTest

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
    QtTest.QTest.keyClicks(dialog.search_edit, "ret")
    qapp.processEvents()

    assert _visible_outcome_names(dialog) == ["retry"]
    assert not dialog.accept_button.isEnabled()

    dialog.outcome_list.setFocus()
    QtTest.QTest.keyClick(dialog.outcome_list, Qt.Key.Key_Space)
    qapp.processEvents()

    assert dialog.accept_button.isEnabled()
    assert dialog.selected_outcomes() == ["retry"]

    QtTest.QTest.mouseClick(dialog.accept_button, Qt.MouseButton.LeftButton)
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
    dialog.close()


def test_transition_outcome_picker_preserves_multiple_checked_outcomes_under_filter(
    qapp,
):
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
    QtTest.QTest.keyClick(dialog.outcome_list, Qt.Key.Key_Space)
    dialog.outcome_list.setCurrentRow(2)
    QtTest.QTest.keyClick(dialog.outcome_list, Qt.Key.Key_Space)
    qapp.processEvents()

    assert dialog.selected_outcomes() == ["done", "failed"]

    dialog.search_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.search_edit, "fa")
    qapp.processEvents()

    assert _visible_outcome_names(dialog) == ["failed"]
    assert dialog.selected_outcomes() == ["done", "failed"]

    dialog.close()
