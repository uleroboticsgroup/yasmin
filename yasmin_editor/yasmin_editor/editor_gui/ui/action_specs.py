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
"""Declarative action definitions for menus and the compact main toolbar.

The toolbar intentionally stays compact. Core actions remain directly visible,
while denser workflows are exposed through menus, shortcuts, and toolbar
overflow buttons.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True, slots=True)
class ActionSpec:
    """Description of one QAction owned by the editor window."""

    attribute_name: str
    text: str
    callback_name: str
    shortcut: str = ""
    checkable: bool = False
    separator_before: bool = False
    tool_tip: str = ""
    status_tip: str = ""


PRIMARY_TOOLBAR_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "new_action",
        "New",
        "new_state_machine",
        shortcut="Ctrl+N",
        tool_tip="Create a new state machine.",
        status_tip="Create a new empty state machine.",
    ),
    ActionSpec(
        "open_action",
        "Open",
        "open_state_machine",
        shortcut="Ctrl+O",
        tool_tip="Open a state machine XML file.",
        status_tip="Open a saved state machine from XML.",
    ),
    ActionSpec(
        "save_action",
        "Save",
        "save_state_machine",
        shortcut="Ctrl+S",
        tool_tip="Save the current state machine.",
        status_tip="Save the current state machine to XML.",
    ),
    ActionSpec(
        "undo_action",
        "Undo",
        "undo_last_action",
        shortcut="Ctrl+Z",
        separator_before=True,
        tool_tip="Undo the last editor change.",
        status_tip="Restore the previous editor snapshot.",
    ),
    ActionSpec(
        "redo_action",
        "Redo",
        "redo_last_action",
        shortcut="Ctrl+Shift+Z",
        tool_tip="Redo the last undone editor change.",
        status_tip="Restore the next editor snapshot.",
    ),
    ActionSpec(
        "add_state_action",
        "Add State",
        "add_state",
        separator_before=True,
        tool_tip="Add a plugin state to the current container.",
        status_tip="Create a new plugin state in the current container.",
    ),
    ActionSpec(
        "edit_current_action",
        "Edit Current Container",
        "edit_current_container",
        separator_before=True,
        tool_tip="Edit the current container properties.",
        status_tip="Edit metadata and outcomes of the current container.",
    ),
    ActionSpec(
        "delete_action",
        "Delete Selected",
        "delete_selected",
        shortcut="Delete",
        tool_tip="Delete the selected items.",
        status_tip="Delete the current scene selection.",
    ),
)

FILE_MENU_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "new_action",
        "New",
        "new_state_machine",
        shortcut="Ctrl+N",
        tool_tip="Create a new state machine.",
        status_tip="Create a new empty state machine.",
    ),
    ActionSpec(
        "open_action",
        "Open",
        "open_state_machine",
        shortcut="Ctrl+O",
        tool_tip="Open a state machine XML file.",
        status_tip="Open a saved state machine from XML.",
    ),
    ActionSpec(
        "save_action",
        "Save",
        "save_state_machine",
        shortcut="Ctrl+S",
        tool_tip="Save the current state machine.",
        status_tip="Save the current state machine to XML.",
    ),
    ActionSpec(
        "save_as_action",
        "Save As",
        "save_state_machine_as",
        shortcut="Ctrl+Shift+S",
        tool_tip="Save the current state machine under a new file name.",
        status_tip="Choose a new XML file path and save the current state machine.",
    ),
)

ADD_MENU_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "add_state_action",
        "Add State",
        "add_state",
        tool_tip="Add a plugin state.",
        status_tip="Create a new plugin state in the current container.",
    ),
    ActionSpec(
        "add_state_machine_action",
        "Add State Machine",
        "add_state_machine",
        tool_tip="Add a nested state machine container.",
        status_tip="Create a nested state machine container.",
    ),
    ActionSpec(
        "add_concurrence_action",
        "Add Concurrence",
        "add_concurrence",
        tool_tip="Add a concurrence container.",
        status_tip="Create a new concurrence container.",
    ),
    ActionSpec(
        "add_final_action",
        "Add Final Outcome",
        "add_final_outcome",
        tool_tip="Add a final outcome to the current container.",
        status_tip="Create a final outcome node in the current container.",
    ),
    ActionSpec(
        "add_text_action",
        "Add Text",
        "add_text_block",
        tool_tip="Add a free-form text note.",
        status_tip="Create an editable text block in the current container.",
    ),
)

EDIT_MENU_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "edit_current_action",
        "Edit Current Container",
        "edit_current_container",
        tool_tip="Edit the current container properties.",
        status_tip="Edit metadata and outcomes of the current container.",
    ),
    ActionSpec(
        "undo_action",
        "Undo",
        "undo_last_action",
        shortcut="Ctrl+Z",
        separator_before=True,
        tool_tip="Undo the last editor change.",
        status_tip="Restore the previous editor snapshot.",
    ),
    ActionSpec(
        "redo_action",
        "Redo",
        "redo_last_action",
        shortcut="Ctrl+Shift+Z",
        tool_tip="Redo the last undone editor change.",
        status_tip="Restore the next editor snapshot.",
    ),
    ActionSpec(
        "delete_action",
        "Delete Selected",
        "delete_selected",
        shortcut="Delete",
        tool_tip="Delete the selected items.",
        status_tip="Delete the current scene selection.",
    ),
    ActionSpec(
        "copy_to_clipboard_action",
        "Copy Selection",
        "start_copy_selection_placement",
        shortcut="Ctrl+C",
        separator_before=True,
        tool_tip="Create a copy of the current selection and place it manually.",
        status_tip="Copy the selected items and attach the copy to the cursor for manual placement.",
    ),
    ActionSpec(
        "extract_selection_action",
        "Extract",
        "extract_selected_items",
        shortcut="Ctrl+E",
        tool_tip="Extract the current selection into a container.",
        status_tip="Extract the selected items into a new nested container.",
    ),
)

VIEW_MENU_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "toggle_clipboard_action",
        "Toggle Shelf",
        "toggle_clipboard_panel",
        shortcut="Ctrl+Shift+B",
        checkable=True,
        tool_tip="Show or hide the shelf dock.",
        status_tip="Toggle visibility of the editor shelf dock.",
    ),
)

HELP_MENU_ACTIONS: tuple[ActionSpec, ...] = (
    ActionSpec(
        "help_action",
        "Help",
        "show_help",
        tool_tip="Open the editor help dialog.",
        status_tip="Show the editor help dialog.",
    ),
)

ALL_ACTION_GROUPS: tuple[tuple[ActionSpec, ...], ...] = (
    PRIMARY_TOOLBAR_ACTIONS,
    FILE_MENU_ACTIONS,
    ADD_MENU_ACTIONS,
    EDIT_MENU_ACTIONS,
    VIEW_MENU_ACTIONS,
    HELP_MENU_ACTIONS,
)


def iter_action_specs(groups: Iterable[tuple[ActionSpec, ...]] = ALL_ACTION_GROUPS):
    """Yield action specs in group order.

    The helper is intentionally tiny, but it gives tests and menu/toolbar wiring
    one canonical way to walk the catalog.
    """

    for group in groups:
        yield from group


def action_specs_by_attribute(
    groups: Iterable[tuple[ActionSpec, ...]] = ALL_ACTION_GROUPS,
) -> dict[str, ActionSpec]:
    """Return the last visible spec for each QAction attribute name."""

    return {spec.attribute_name: spec for spec in iter_action_specs(groups)}
