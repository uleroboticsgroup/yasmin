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

import os
from dataclasses import dataclass
from typing import Optional

from yasmin_editor.qt_compat import QtGui

YASMIN_EDITOR_THEME_ENV = "YASMIN_EDITOR_THEME"
DEFAULT_PALETTE_NAME = "default"
DARKMODE_PALETTE_NAME = "darkmode"


@dataclass(frozen=True)
class EditorPalette:
    """Complete color palette used by the editor UI and scene items."""

    background: QtGui.QColor
    text_primary: QtGui.QColor
    text_secondary: QtGui.QColor
    ui_window_bg: QtGui.QColor
    ui_panel_bg: QtGui.QColor
    ui_panel_alt_bg: QtGui.QColor
    ui_input_bg: QtGui.QColor
    ui_button_bg: QtGui.QColor
    ui_button_hover_bg: QtGui.QColor
    ui_button_pressed_bg: QtGui.QColor
    ui_border: QtGui.QColor
    ui_selection_bg: QtGui.QColor
    ui_selection_text: QtGui.QColor
    ui_tooltip_bg: QtGui.QColor
    ui_tooltip_text: QtGui.QColor
    shell_bg: QtGui.QColor
    shell_text: QtGui.QColor
    shell_border: QtGui.QColor
    shell_selection_bg: QtGui.QColor
    shell_selection_text: QtGui.QColor
    shell_prompt_in: QtGui.QColor
    shell_prompt_out: QtGui.QColor
    shell_comment: QtGui.QColor
    shell_keyword: QtGui.QColor
    shell_string: QtGui.QColor
    shell_number: QtGui.QColor
    shell_error: QtGui.QColor
    state_python_fill: QtGui.QColor
    state_cpp_fill: QtGui.QColor
    state_xml_fill: QtGui.QColor
    state_pen: QtGui.QColor
    container_xml_fill: QtGui.QColor
    container_xml_pen: QtGui.QColor
    container_concurrence_fill: QtGui.QColor
    container_concurrence_pen: QtGui.QColor
    container_orthogonal_fill: QtGui.QColor
    container_orthogonal_pen: QtGui.QColor
    container_state_machine_fill: QtGui.QColor
    container_state_machine_pen: QtGui.QColor
    final_outcome_fill: QtGui.QColor
    final_outcome_pen: QtGui.QColor
    selection_pen: QtGui.QColor
    blackboard_highlight_pen: QtGui.QColor
    blackboard_highlight_fill: QtGui.QColor
    start_indicator_connector: QtGui.QColor
    start_indicator_outer_fill: QtGui.QColor
    start_indicator_outer_pen: QtGui.QColor
    start_indicator_inner_fill: QtGui.QColor
    start_indicator_inner_pen: QtGui.QColor
    start_indicator_arrow: QtGui.QColor
    start_indicator_label: QtGui.QColor
    runtime_highlight_pen: QtGui.QColor
    runtime_highlight_fill: QtGui.QColor
    runtime_transition_pen: QtGui.QColor
    runtime_transition_label_bg: QtGui.QColor
    runtime_canvas_border: QtGui.QColor
    runtime_mode_button_bg: QtGui.QColor
    runtime_mode_button_text: QtGui.QColor
    runtime_log_default: QtGui.QColor
    runtime_log_status: QtGui.QColor
    runtime_log_info: QtGui.QColor
    runtime_log_debug: QtGui.QColor
    runtime_log_warn: QtGui.QColor
    runtime_log_error: QtGui.QColor
    runtime_log_system: QtGui.QColor
    connection_line: QtGui.QColor
    connection_selected: QtGui.QColor
    connection_label_bg: QtGui.QColor
    connection_label_pen: QtGui.QColor
    connection_label_text: QtGui.QColor
    connection_port_fill: QtGui.QColor
    connection_port_pen: QtGui.QColor
    temp_connection: QtGui.QColor

    def state_fill(self, plugin_type: Optional[str]) -> QtGui.QColor:
        """Return the fill color for a state based on the plugin type."""
        if plugin_type == "python":
            return self.state_python_fill
        if plugin_type == "cpp":
            return self.state_cpp_fill
        return self.state_xml_fill


def normalize_palette_name(value: str) -> str:
    """Normalize user supplied palette names to supported palette keys."""
    normalized = value.strip().lower().replace("-", "").replace("_", "")
    if normalized in {"dark", DARKMODE_PALETTE_NAME}:
        return DARKMODE_PALETTE_NAME
    return DEFAULT_PALETTE_NAME


def get_palette_name_from_env(env_var: str = YASMIN_EDITOR_THEME_ENV) -> str:
    """Resolve the active palette name from the configured environment variable."""
    return normalize_palette_name(os.getenv(env_var, DEFAULT_PALETTE_NAME))


PALETTE_NAME = get_palette_name_from_env()
