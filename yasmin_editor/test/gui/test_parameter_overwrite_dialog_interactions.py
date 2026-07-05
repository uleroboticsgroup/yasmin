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

from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
    ParameterOverwriteDialog,
)


def _dialog_button(dialog, standard_button):
    # Resolve one standard dialog button so the test can click the same control
    # a user would activate in the running application.
    button_box = dialog.findChild(QtWidgets.QDialogButtonBox)
    assert button_box is not None
    button = button_box.button(standard_button)
    assert button is not None
    return button


def test_parameter_overwrite_dialog_validates_and_collects_user_input(qapp, monkeypatch):
    # The dialog must reject incomplete input, enable the default-value field
    # only when a default type is selected, and return normalized overwrite
    # data after valid user input.
    warnings = []

    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "warning",
        lambda *_args: warnings.append(_args[2]),
    )

    dialog = ParameterOverwriteDialog(
        declared_parameters=[
            {"name": "timeout", "description": "planner timeout"},
            {"name": "mode", "description": "planner mode"},
        ]
    )
    dialog.show()
    qapp.processEvents()

    assert not dialog.default_value_edit.isEnabled()

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert warnings == ["Parameter name is required!"]
    assert dialog.result() == QtWidgets.QDialog.DialogCode.Rejected

    dialog.name_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.name_edit, "planner")
    dialog.default_type_combo.setCurrentText("int")
    qapp.processEvents()

    assert dialog.default_value_edit.isEnabled()
    assert dialog.child_param_combo.currentData() == "timeout"

    dialog.description_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.description_edit, "override timeout")
    dialog.default_value_edit.setFocus()
    QtTest.QTest.keyClicks(dialog.default_value_edit, "15")

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Ok),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Accepted
    assert dialog.get_parameter_data() == {
        "name": "planner",
        "child_parameter": "timeout",
        "description": "override timeout",
        "default_type": "int",
        "default_value": "15",
    }

    dialog.close()


def test_parameter_overwrite_dialog_readonly_exposes_existing_data_without_editing(
    qapp,
):
    # Readonly mode must keep the stored values visible while disabling every
    # editing control and exposing only a close button.
    dialog = ParameterOverwriteDialog(
        declared_parameters=[{"name": "timeout", "description": "planner timeout"}],
        param_data={
            "name": "planner",
            "child_parameter": "timeout",
            "description": "existing overwrite",
            "default_type": "int",
            "default_value": "30",
        },
        readonly=True,
    )
    dialog.show()
    qapp.processEvents()

    assert dialog.name_edit.isReadOnly()
    assert not dialog.child_param_combo.isEnabled()
    assert dialog.description_edit.isReadOnly()
    assert not dialog.default_type_combo.isEnabled()
    assert dialog.default_value_edit.isReadOnly()
    assert _dialog_button(
        dialog, QtWidgets.QDialogButtonBox.StandardButton.Close
    ).isEnabled()

    assert dialog.get_parameter_data() == {
        "name": "planner",
        "child_parameter": "timeout",
        "description": "existing overwrite",
        "default_type": "int",
        "default_value": "30",
    }

    QtTest.QTest.mouseClick(
        _dialog_button(dialog, QtWidgets.QDialogButtonBox.StandardButton.Close),
        Qt.MouseButton.LeftButton,
    )
    qapp.processEvents()

    assert dialog.result() == QtWidgets.QDialog.DialogCode.Rejected
    dialog.close()
