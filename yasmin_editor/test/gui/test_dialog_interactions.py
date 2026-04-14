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

# GUI tests for the metadata dialogs used by the editor.
#
# Each test opens the real dialog, sends keyboard or mouse input, and checks
# that the dialog returns the expected structured data.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QDialogButtonBox, QPushButton, QTableWidgetItem

from yasmin_editor.editor_gui.dialogs.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.outcome_description_dialog import (
    OutcomeDescriptionDialog,
)
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import StateMachineDialog


def _dialog_button(dialog, standard_button):
    # Resolve one standard button from the button box so the test can click it.
    button_box = dialog.findChild(QDialogButtonBox)
    assert button_box is not None
    button = button_box.button(standard_button)
    assert button is not None
    return button


def _find_button_by_text(dialog, text: str) -> QPushButton:
    # The row-creation button is identified by text because it is not a
    # standard dialog button.
    for button in dialog.findChildren(QPushButton):
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
    QTest.keyClicks(dialog.name_edit, "WorkerContainer")

    QTest.mouseClick(_find_button_by_text(dialog, "Add Row"), Qt.LeftButton)
    qapp.processEvents()

    row = dialog.remappings_table.rowCount() - 1
    dialog.remappings_table.setItem(row, 0, QTableWidgetItem("output"))
    dialog.remappings_table.setItem(row, 1, QTableWidgetItem("bb_output"))
    dialog.description_edit.setFocus()
    QTest.keyClicks(dialog.description_edit, "Nested editor container")

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
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
    QTest.keyClicks(dialog.name_edit, "ParallelWorker")
    dialog.default_outcome_combo.setCurrentText("done")
    qapp.processEvents()

    QTest.mouseClick(_find_button_by_text(dialog, "Add Row"), Qt.LeftButton)
    qapp.processEvents()
    row = dialog.remappings_table.rowCount() - 1
    dialog.remappings_table.setItem(row, 0, QTableWidgetItem("result"))
    dialog.remappings_table.setItem(row, 1, QTableWidgetItem("bb_result"))

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
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
    QTest.keyClick(dialog.name_edit, Qt.Key_Delete)
    QTest.keyClicks(dialog.name_edit, "finished")

    dialog.description_edit.selectAll()
    QTest.keyClick(dialog.description_edit, Qt.Key_Delete)
    QTest.keyClicks(dialog.description_edit, "final transition target")

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
    assert dialog.get_outcome_name() == "finished"
    assert dialog.get_description() == "final transition target"

    dialog.close()
