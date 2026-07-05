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

from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import BlackboardKeyDialog


def _dialog_button(dialog, standard_button):
    # Resolve one standard dialog button so the test can click the same control
    # a user would use in the running application.
    button_box = dialog.findChild(QtWidgets.QDialogButtonBox)
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
    QtTest.QTest.keyClicks(dialog.name_edit, "goal_pose")
    dialog.type_combo.setCurrentText("in/out")
    dialog.default_type_combo.setCurrentText("list[int]")
    qapp.processEvents()

    assert dialog.default_value_edit.isEnabled()

    dialog.description_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.description_edit, "goal indices")
    dialog.default_value_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.default_value_edit, "[1, 2, 3]")

    # Accept the dialog and verify the resulting payload.
    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
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

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.get_key_data()["default_type"] == ""
    assert dialog.get_key_data()["default_value"] == ""

    dialog.close()
