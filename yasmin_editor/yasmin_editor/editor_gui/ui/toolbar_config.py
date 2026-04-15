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
"""Pure toolbar configuration shared by the toolbar builder and tests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ToolbarMenuSpec:
    """Description of a compact toolbar drop-down button."""

    object_name: str
    button_attribute_name: str
    text: str
    action_attributes: tuple[str, ...]
    tool_tip: str
    separator_before: bool = False


ADD_TOOLBAR_MENU = ToolbarMenuSpec(
    object_name="toolbarAddMenuButton",
    button_attribute_name="toolbar_add_menu_button",
    text="More Add",
    action_attributes=(
        "add_state_machine_action",
        "add_concurrence_action",
        "add_final_action",
        "add_text_action",
    ),
    tool_tip="Open additional add actions.",
)

SELECTION_TOOLBAR_MENU = ToolbarMenuSpec(
    object_name="toolbarSelectionMenuButton",
    button_attribute_name="toolbar_selection_menu_button",
    text="Selection",
    action_attributes=(
        "copy_to_clipboard_action",
        "extract_selection_action",
    ),
    tool_tip="Open selection and shelf actions.",
)

TOOLBAR_MENU_SPECS: tuple[ToolbarMenuSpec, ...] = (
    ADD_TOOLBAR_MENU,
    SELECTION_TOOLBAR_MENU,
)
