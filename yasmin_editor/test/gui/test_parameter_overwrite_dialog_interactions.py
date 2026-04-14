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

# GUI tests for the parameter overwrite dialog.
#
# These checks drive the real dialog widgets to verify validation, default-value
# enablement, and readonly presentation in headless Qt test runs.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QDialogButtonBox, QMessageBox

from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
    ParameterOverwriteDialog,
)


def _dialog_button(dialog, standard_button):
    # Resolve one standard dialog button so the test can click the same control
    # a user would activate in the running application.
    button_box = dialog.findChild(QDialogButtonBox)
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
        QMessageBox,
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

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert warnings == ["Parameter name is required!"]
    assert dialog.result() == dialog.Rejected

    dialog.name_edit.setFocus()
    QTest.keyClicks(dialog.name_edit, "planner")
    dialog.default_type_combo.setCurrentText("int")
    qapp.processEvents()

    assert dialog.default_value_edit.isEnabled()
    assert dialog.child_param_combo.currentData() == "timeout"

    dialog.description_edit.setFocus()
    QTest.keyClicks(dialog.description_edit, "override timeout")
    dialog.default_value_edit.setFocus()
    QTest.keyClicks(dialog.default_value_edit, "15")

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Ok), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Accepted
    assert dialog.get_parameter_data() == {
        "name": "planner",
        "child_parameter": "timeout",
        "description": "override timeout",
        "default_type": "int",
        "default_value": "15",
    }

    dialog.close()


def test_parameter_overwrite_dialog_readonly_exposes_existing_data_without_editing(qapp):
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
    assert _dialog_button(dialog, QDialogButtonBox.Close).isEnabled()

    assert dialog.get_parameter_data() == {
        "name": "planner",
        "child_parameter": "timeout",
        "description": "existing overwrite",
        "default_type": "int",
        "default_value": "30",
    }

    QTest.mouseClick(_dialog_button(dialog, QDialogButtonBox.Close), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.result() == dialog.Rejected
    dialog.close()
