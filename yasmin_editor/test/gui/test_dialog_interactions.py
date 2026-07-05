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

from yasmin_editor.qt_compat import Qt
from yasmin_editor.qt_compat import QtTest
from yasmin_editor.qt_compat import QtWidgets

from yasmin_editor.editor_gui.dialogs.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.outcome_description_dialog import (
    OutcomeDescriptionDialog,
)
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import StateMachineDialog


def _dialog_button(dialog, standard_button):
    # Resolve one standard button from the button box so the test can click it.
    button_box = dialog.findChild(QtWidgets.QDialogButtonBox)
    assert button_box is not None
    button = button_box.button(standard_button)
    assert button is not None
    return button


def _find_button_by_text(dialog, text: str) -> QtWidgets.QPushButton:
    # The row-creation button is identified by text because it is not a
    # standard dialog button.
    for button in dialog.findChildren(QtWidgets.QPushButton):
        if button.text() == text:
            return button
    raise AssertionError(f"Button '{text}' not found")


def test_state_machine_dialog_collects_user_input(qapp):
    # Create a nested container configuration with one extra remapping row.
    dialog = StateMachineDialog(
        outcomes=["done", "failed"],
        remappings={"input": "bb_input"},
    )
    dialog.show()
    qapp.processEvents()

    # Fill the dialog through its real widgets.
    dialog.name_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.name_edit, "WorkerContainer")

    QtTest.QTest.mouseClick(
        _find_button_by_text(dialog, "Add Row"), Qt.MouseButton.LeftButton
    )
    qapp.processEvents()

    row = dialog.remappings_table.rowCount() - 1
    dialog.remappings_table.setItem(row, 0, QtWidgets.QTableWidgetItem("output"))
    dialog.remappings_table.setItem(row, 1, QtWidgets.QTableWidgetItem("bb_output"))
    dialog.description_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.description_edit, "Nested editor container")

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
    assert dialog.get_state_machine_data() == (
        "WorkerContainer",
        ["done", "failed"],
        None,
        {"input": "bb_input", "output": "bb_output"},
        "Nested editor container",
        [],
    )

    dialog.close()


def test_concurrence_dialog_collects_default_outcome_and_remappings(qapp):
    # Change the default outcome and add another remapping entry.
    dialog = ConcurrenceDialog(
        outcomes=["done", "failed"],
        final_outcomes=["done", "failed"],
        default_outcome="failed",
        remappings={"source": "bb_source"},
        description="parallel block",
    )
    dialog.show()
    qapp.processEvents()

    dialog.name_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.name_edit, "ParallelWorker")
    dialog.default_outcome_combo.setCurrentText("done")
    qapp.processEvents()

    QtTest.QTest.mouseClick(
        _find_button_by_text(dialog, "Add Row"), Qt.MouseButton.LeftButton
    )
    qapp.processEvents()
    row = dialog.remappings_table.rowCount() - 1
    dialog.remappings_table.setItem(row, 0, QtWidgets.QTableWidgetItem("result"))
    dialog.remappings_table.setItem(row, 1, QtWidgets.QTableWidgetItem("bb_result"))

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
    assert dialog.get_concurrence_data() == (
        "ParallelWorker",
        ["done", "failed"],
        "done",
        {"source": "bb_source", "result": "bb_result"},
        "parallel block",
        [],
    )

    dialog.close()


def test_outcome_description_dialog_round_trips_user_edits(qapp):
    # Rename the outcome and update its description before accepting the dialog.
    dialog = OutcomeDescriptionDialog("done", "old description")
    dialog.show()
    qapp.processEvents()

    dialog.name_edit.selectAll()
    QtTest.QTest.keyClick(dialog.name_edit, Qt.Key.Key_Delete)
    QtTest.QTest.keyClicks(dialog.name_edit, "finished")

    dialog.description_edit.selectAll()
    QtTest.QTest.keyClick(dialog.description_edit, Qt.Key.Key_Delete)
    QtTest.QTest.keyClicks(dialog.description_edit, "final transition target")

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
    assert dialog.get_outcome_name() == "finished"
    assert dialog.get_description() == "final transition target"

    dialog.close()
