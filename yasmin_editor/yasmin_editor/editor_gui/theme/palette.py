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

"""Palette model and palette selection helpers for the editor theme."""

import os
from dataclasses import dataclass
from typing import Optional

from PyQt5.QtGui import QColor

YASMIN_EDITOR_COLOR_ENV = "YASMIN_EDITOR_COLOR"
DEFAULT_PALETTE_NAME = "default"
DARKMODE_PALETTE_NAME = "darkmode"


@dataclass(frozen=True)
class EditorPalette:
    """Complete color palette used by the editor UI and scene items."""

    background: QColor
    text_primary: QColor
    text_secondary: QColor
    ui_window_bg: QColor
    ui_panel_bg: QColor
    ui_panel_alt_bg: QColor
    ui_input_bg: QColor
    ui_button_bg: QColor
    ui_button_hover_bg: QColor
    ui_button_pressed_bg: QColor
    ui_border: QColor
    ui_selection_bg: QColor
    ui_selection_text: QColor
    ui_tooltip_bg: QColor
    ui_tooltip_text: QColor
    state_python_fill: QColor
    state_cpp_fill: QColor
    state_xml_fill: QColor
    state_pen: QColor
    container_xml_fill: QColor
    container_xml_pen: QColor
    container_concurrence_fill: QColor
    container_concurrence_pen: QColor
    container_state_machine_fill: QColor
    container_state_machine_pen: QColor
    final_outcome_fill: QColor
    final_outcome_pen: QColor
    selection_pen: QColor
    blackboard_highlight_pen: QColor
    blackboard_highlight_fill: QColor
    runtime_highlight_pen: QColor
    runtime_highlight_fill: QColor
    runtime_transition_pen: QColor
    runtime_transition_label_bg: QColor
    runtime_canvas_border: QColor
    runtime_mode_button_bg: QColor
    runtime_mode_button_text: QColor
    runtime_log_default: QColor
    runtime_log_status: QColor
    runtime_log_info: QColor
    runtime_log_debug: QColor
    runtime_log_warn: QColor
    runtime_log_error: QColor
    runtime_log_system: QColor
    connection_line: QColor
    connection_selected: QColor
    connection_label_bg: QColor
    connection_label_pen: QColor
    connection_label_text: QColor
    connection_port_fill: QColor
    connection_port_pen: QColor
    temp_connection: QColor

    def state_fill(self, plugin_type: Optional[str]) -> QColor:
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


def get_palette_name_from_env(env_var: str = YASMIN_EDITOR_COLOR_ENV) -> str:
    """Resolve the active palette name from the configured environment variable."""
    return normalize_palette_name(os.getenv(env_var, DEFAULT_PALETTE_NAME))


PALETTE_NAME = get_palette_name_from_env()
