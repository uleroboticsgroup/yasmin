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

# GUI tests for the state-properties dialog.
#
# These checks verify that the dialog updates the plugin list when the selected
# state type changes and that button-driven parameter-overwrite insertion ends
# up in the dialog result payload.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QPushButton, QTableWidgetItem

from gui_test_support import FakePluginInfo
from yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog import (
    ParameterOverwriteDialog,
)
from yasmin_editor.editor_gui.dialogs.state_properties_dialog import (
    StatePropertiesDialog,
)


def _find_button_by_text(dialog, text: str) -> QPushButton:
    # The toolbar-like table buttons are identified by text because they are
    # not standard buttons from a dialog button box.
    for button in dialog.findChildren(QPushButton):
        if button.text() == text:
            return button
    raise AssertionError(f"Button '{text}' not found")


def _make_plugin(plugin_type: str, module: str, class_name: str) -> FakePluginInfo:
    # Build one fake plugin with enough metadata for the description panel and
    # result payload assertions.
    plugin = FakePluginInfo(plugin_type=plugin_type, module=module, class_name=class_name)
    plugin.description = f"{class_name} description"
    plugin.parameters = [
        {
            "name": "timeout",
            "description": "planner timeout",
            "type": "int",
            "default_value": "10",
        }
    ]
    plugin.input_keys = [
        {
            "name": "goal",
            "description": "target pose",
            "type": "str",
        }
    ]
    plugin.output_keys = [
        {
            "name": "result",
            "description": "planner result",
            "type": "str",
        }
    ]
    return plugin


def test_state_properties_dialog_switches_plugin_types_and_updates_description(qapp):
    # Switching the type selector must rebuild the plugin combo and refresh the
    # read-only description text from the newly selected plugin metadata.
    python_plugin = _make_plugin("python", "demo_pkg.alpha", "AlphaState")
    cpp_plugin = _make_plugin("cpp", "demo_pkg.cpp", "CppState")
    xml_plugin = _make_plugin("xml", "demo_pkg.xml", "XmlState")
    xml_plugin.package_name = "demo_pkg"
    xml_plugin.file_name = "/tmp/demo_state.xml"

    dialog = StatePropertiesDialog(
        available_plugins=[python_plugin, cpp_plugin, xml_plugin],
    )
    dialog.show()
    qapp.processEvents()

    assert dialog.plugin_combo.count() == 1
    assert dialog.plugin_combo.itemText(0) == "demo_pkg.alpha.AlphaState"
    assert "AlphaState description" in dialog.description_edit.toPlainText()
    assert "Outcomes:" in dialog.description_edit.toPlainText()
    assert "Parameters:" in dialog.description_edit.toPlainText()

    dialog.type_combo.setCurrentText("XML File")
    qapp.processEvents()

    assert dialog.plugin_combo.count() == 1
    assert dialog.plugin_combo.itemText(0) == "demo_pkg::demo_state.xml"
    assert "XmlState description" in dialog.description_edit.toPlainText()

    dialog.type_combo.setCurrentText("C++ State")
    qapp.processEvents()

    assert dialog.plugin_combo.count() == 1
    assert dialog.plugin_combo.itemText(0) == "CppState"
    assert "CppState description" in dialog.description_edit.toPlainText()

    dialog.close()


def test_state_properties_dialog_adds_parameter_overwrite_via_button_flow(
    qapp, monkeypatch
):
    # Clicking the add-overwrite button must append the nested dialog result to
    # the table and include it in the final structured state payload.
    plugin = _make_plugin("python", "demo_pkg.alpha", "AlphaState")

    def fake_exec(self):
        self.name_edit.setText("planner")
        self.child_param_combo.setCurrentIndex(0)
        self.description_edit.setPlainText("override timeout")
        self.default_type_combo.setCurrentText("int")
        self.default_value_edit.setText("25")
        return self.Accepted

    monkeypatch.setattr(ParameterOverwriteDialog, "exec_", fake_exec)

    dialog = StatePropertiesDialog(
        available_plugins=[plugin],
        remappings={"input": "bb_input"},
    )
    dialog.show()
    qapp.processEvents()

    dialog.name_edit.setFocus()
    QTest.keyClicks(dialog.name_edit, "Worker")

    QTest.mouseClick(_find_button_by_text(dialog, "Add Param Overwrite"), Qt.LeftButton)
    qapp.processEvents()

    assert dialog.parameter_table.rowCount() == 1
    assert dialog.parameter_table.item(0, 0).text() == "planner"
    assert dialog.parameter_table.item(0, 1).text() == "timeout"
    assert dialog.parameter_table.item(0, 2).text() == "int"
    assert dialog.parameter_table.item(0, 3).text() == "25"

    QTest.mouseClick(_find_button_by_text(dialog, "Add Row"), Qt.LeftButton)
    qapp.processEvents()

    row = dialog.remappings_table.rowCount() - 1
    dialog.remappings_table.setItem(row, 0, QTableWidgetItem("output"))
    dialog.remappings_table.setItem(row, 1, QTableWidgetItem("bb_output"))

    state_name, plugin_info, outcomes, remappings, description, defaults, overwrites = (
        dialog.get_state_data()
    )

    assert state_name == "Worker"
    assert plugin_info == plugin
    assert outcomes == ["done", "failed"]
    assert remappings == {"input": "bb_input", "output": "bb_output"}
    assert "AlphaState description" in description
    assert defaults == []
    assert overwrites == [
        {
            "name": "planner",
            "child_parameter": "timeout",
            "description": "override timeout",
            "default_type": "int",
            "default_value": "25",
        }
    ]

    dialog.close()
