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
from yasmin_editor.dataclass_compat import dataclass


@dataclass(frozen=True, slots=True)
class ToolbarMenuSpec:
    """Description of a compact toolbar drop-down button."""

    object_name: str
    button_attribute_name: str
    text: str
    action_attributes: Tuple[str, ...]
    tool_tip: str
    separator_before: bool = False


ADD_TOOLBAR_MENU = ToolbarMenuSpec(
    object_name="toolbarAddMenuButton",
    button_attribute_name="toolbar_add_menu_button",
    text="More Add",
    action_attributes=(
        "add_state_machine_action",
        "add_concurrence_action",
        "add_orthogonal_state_action",
        "add_join_state_action",
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

TOOLBAR_MENU_SPECS: Tuple[ToolbarMenuSpec, ...] = (
    ADD_TOOLBAR_MENU,
    SELECTION_TOOLBAR_MENU,
)
