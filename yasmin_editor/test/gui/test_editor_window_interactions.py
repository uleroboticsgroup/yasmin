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

# GUI tests for interactive behavior in the main editor window.
#
# The tests in this module use the real widgets, actions, and shortcuts of the
# editor while keeping runtime dependencies stubbed out.

import pytest

pytest.importorskip("PyQt5.QtCore")
pytest.importorskip("PyQt5.QtTest")
pytest.importorskip("PyQt5.QtWidgets")

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest

from gui_test_support import FakePluginInfo


def _visible_list_items(list_widget):
    # Return the visible labels of a generic QListWidget after filtering.
    return [
        list_widget.item(index).text()
        for index in range(list_widget.count())
        if not list_widget.item(index).isHidden()
    ]


def _visible_blackboard_key_names(list_widget):
    # Return the visible logical key names so the test is independent from the
    # exact sidebar label formatting.
    return [
        (list_widget.item(index).data(Qt.UserRole) or {}).get("name", "")
        for index in range(list_widget.count())
        if not list_widget.item(index).isHidden()
    ]


def test_editor_window_updates_metadata_toggles_shelf_and_filters_plugins(
    editor_window, qapp
):
    # Open and close the clipboard shelf, then verify metadata editing and
    # plugin-list filtering in the main window.
    editor = editor_window

    assert not editor.clipboard_dock.isVisible()

    # Toggle the clipboard shelf through the toolbar action and through the
    # dedicated shelf button.
    editor.toggle_clipboard_action.trigger()
    qapp.processEvents()
    assert editor.clipboard_dock.isVisible()

    QTest.mouseClick(editor.clipboard_panel_toggle_button, Qt.LeftButton)
    qapp.processEvents()
    assert not editor.clipboard_dock.isVisible()

    # Edit the root container metadata through the visible text fields.
    editor.root_sm_name_edit.setFocus()
    QTest.keyClicks(editor.root_sm_name_edit, "DemoRoot")
    editor.root_sm_description_edit.setFocus()
    QTest.keyClicks(editor.root_sm_description_edit, "interactive ui test")
    qapp.processEvents()

    assert editor.root_model.name == "DemoRoot"
    assert editor.root_model.description == "interactive ui test"

    assert editor.python_list.count() == 2

    # Narrow the plugin list and verify that only the matching plugin remains
    # visible.
    editor.python_filter.setFocus()
    QTest.keyClicks(editor.python_filter, "beta")
    qapp.processEvents()

    assert _visible_list_items(editor.python_list) == ["demo_pkg.beta.BetaState"]


def test_editor_window_blackboard_filter_and_visibility_controls(editor_window, qapp):
    # Add persistent blackboard entries, toggle the visibility controls, and
    # verify that the sidebar filter works on the resulting list.
    editor = editor_window

    # Seed the editor with persistent defaults so the keys remain visible even
    # without a live state using them yet.
    editor.add_root_default_row_with_data(
        {
            "key": "visible_key",
            "type": "str",
            "value": "ready",
            "description": "visible",
        }
    )
    editor.add_root_default_row_with_data(
        {
            "key": ".hidden_key",
            "type": "str",
            "value": "secret",
            "description": "hidden",
        }
    )
    qapp.processEvents()

    assert _visible_blackboard_key_names(editor.blackboard_list) == ["visible_key"]
    assert editor.highlight_blackboard_btn.text() == "Highlight: On"
    assert editor.show_hidden_blackboard_btn.text() == "Hidden: Off"

    QTest.mouseClick(editor.highlight_blackboard_btn, Qt.LeftButton)
    qapp.processEvents()
    assert editor.highlight_blackboard_btn.text() == "Highlight: Off"

    QTest.mouseClick(editor.show_hidden_blackboard_btn, Qt.LeftButton)
    qapp.processEvents()
    assert editor.show_hidden_blackboard_btn.text() == "Hidden: On"
    assert _visible_blackboard_key_names(editor.blackboard_list) == [
        ".hidden_key",
        "visible_key",
    ]

    # Filter the sidebar down to the visible key and verify that the hidden
    # entry drops out again.
    editor.blackboard_filter.setFocus()
    QTest.keyClicks(editor.blackboard_filter, "visible")
    qapp.processEvents()
    assert _visible_blackboard_key_names(editor.blackboard_list) == ["visible_key"]


def test_editor_window_adds_text_block_and_cancels_pending_items(
    editor_window, monkeypatch, qapp
):
    # Start placing a text block and a final outcome, then cancel both pending
    # placements with Escape.
    editor = editor_window

    # The text action creates a model entry and leaves the canvas in placement
    # mode until the interaction is either completed or canceled.
    editor.add_text_action.trigger()
    qapp.processEvents()

    assert len(editor.current_container_model.text_blocks) == 1
    assert editor.canvas.pending_placement_item is not None

    editor.canvas.setFocus()
    QTest.keyClick(editor.canvas.viewport(), Qt.Key_Escape)
    qapp.processEvents()

    assert editor.canvas.pending_placement_item is None
    assert len(editor.text_blocks) == 0

    # Stub the outcome-name dialog so the test can enter the placement flow
    # without opening an additional modal dialog.
    monkeypatch.setattr(
        "PyQt5.QtWidgets.QInputDialog.getText",
        lambda *_args, **_kwargs: ("finished", True),
    )
    editor.add_final_action.trigger()
    qapp.processEvents()

    assert any(
        outcome.name == "finished" for outcome in editor.current_container_model.outcomes
    )
    assert editor.canvas.pending_placement_item is not None

    editor.canvas.setFocus()
    QTest.keyClick(editor.canvas.viewport(), Qt.Key_Escape)
    qapp.processEvents()

    assert editor.canvas.pending_placement_item is None
    assert all(
        outcome.name != "finished" for outcome in editor.current_container_model.outcomes
    )


def test_editor_window_pending_state_updates_start_state_selector(editor_window, qapp):
    # Creating the first state should temporarily make it the start state, and
    # canceling the pending placement should revert that change again.
    editor = editor_window

    editor.create_state_node(
        name="worker",
        plugin_info=FakePluginInfo(module="demo_pkg.alpha", class_name="AlphaState"),
    )
    qapp.processEvents()

    assert "worker" in editor.current_container_model.states
    assert editor.current_container_model.start_state == "worker"
    assert editor.start_state_combo.currentText() == "worker"
    assert editor.canvas.pending_placement_item is not None

    editor.canvas.setFocus()
    QTest.keyClick(editor.canvas.viewport(), Qt.Key_Escape)
    qapp.processEvents()

    assert "worker" not in editor.current_container_model.states
    assert editor.current_container_model.start_state is None
    assert editor.start_state_combo.currentText() == "(None)"
