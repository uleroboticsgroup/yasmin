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
"""Qt-free editor action availability helpers."""

from __future__ import annotations


def build_editor_action_enabled_map(
    *,
    read_only_mode: bool,
    has_selection: bool,
    has_state_selection: bool,
    clipboard_has_content: bool,
) -> dict[str, bool]:
    """Return enabled states for the editor actions that depend on context."""

    editable = not read_only_mode
    return {
        "add_state_action": editable,
        "add_state_machine_action": editable,
        "add_concurrence_action": editable,
        "add_final_action": editable,
        "add_text_action": editable,
        "delete_action": editable and has_selection,
        "copy_to_clipboard_action": editable and has_selection,
        "extract_selection_action": editable and has_state_selection,
    }


def toolbar_menu_enabled(
    action_attributes: tuple[str, ...],
    enabled_map: dict[str, bool],
) -> bool:
    """Return whether one toolbar drop-down button should stay enabled."""

    return any(
        enabled_map.get(attribute_name, True) for attribute_name in action_attributes
    )
