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

from __future__ import annotations
from typing import Tuple


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
        "add_orthogonal_state_action": editable,
        "add_join_state_action": editable,
        "add_final_action": editable,
        "add_text_action": editable,
        "delete_action": editable and has_selection,
        "copy_to_clipboard_action": editable and has_selection,
        "extract_selection_action": editable and has_state_selection,
    }


def toolbar_menu_enabled(
    action_attributes: Tuple[str, ...],
    enabled_map: dict[str, bool],
) -> bool:
    """Return whether one toolbar drop-down button should stay enabled."""

    return any(
        enabled_map.get(attribute_name, True) for attribute_name in action_attributes
    )
