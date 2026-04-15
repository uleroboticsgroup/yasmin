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

# GUI tests for the blackboard key dialog.
#
# These checks verify that user input is written back into the dialog result and
# that key-mode changes update the default-value widgets accordingly.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QDialogButtonBox

from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import BlackboardKeyDialog


def _dialog_button(dialog, standard_button):
    # Resolve one standard dialog button so the test can click the same control
    # a user would use in the running application.
    button_box = dialog.findChild(QDialogButtonBox)
    assert button_box is not None
    button = button_box.button(standard_button)
    assert button is not None
    return button


def test_blackboard_key_dialog_collects_defaults_for_input_key(qapp):
    # Enter a key with defaults and accept the dialog through the OK button.
    dialog = BlackboardKeyDialog()
    dialog.show()
    qapp.processEvents()

    # Fill the editable fields exactly like a user would do in the dialog.
    dialog.name_edit.setFocus()
    QTest.keyClicks(dialog.name_edit, "goal_pose")
    dialog.type_combo.setCurrentText("in/out")
    dialog.default_type_combo.setCurrentText("list[int]")
    qapp.processEvents()

    assert dialog.default_value_edit.isEnabled()

    dialog.description_edit.setFocus()
    QTest.keyClicks(dialog.description_edit, "goal indices")
    dialog.default_value_edit.setFocus()
    QTest.keyClicks(dialog.default_value_edit, "[1, 2, 3]")

    # Accept the dialog and verify the resulting payload.
    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
    assert dialog.get_key_data() == {
        "name": "goal_pose",
        "key_type": "in/out",
        "description": "goal indices",
        "default_type": "list[int]",
        "default_value": "[1, 2, 3]",
    }

    dialog.close()


def test_blackboard_key_dialog_disables_and_clears_defaults_for_output_only_key(qapp):
    # Switching to an output-only key must disable and clear input defaults.
    dialog = BlackboardKeyDialog(
        {
            "name": "result",
            "key_type": "in",
            "default_type": "str",
            "default_value": "done",
        }
    )
    dialog.show()
    qapp.processEvents()

    assert dialog.default_value_edit.isEnabled()

    # Change the key mode and verify that the default editors are disabled.
    dialog.type_combo.setCurrentText("out")
    qapp.processEvents()

    assert not dialog.default_type_combo.isEnabled()
    assert not dialog.default_value_edit.isEnabled()
    assert dialog.default_value_edit.text() == ""

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.get_key_data()["default_type"] == ""
    assert dialog.get_key_data()["default_value"] == ""

    dialog.close()
