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

"""Static editor palette definitions.

This module intentionally contains only palette data to keep the theme setup
separate from stylesheet generation and environment handling.
"""

from PyQt5.QtGui import QColor

from yasmin_editor.editor_gui.theme.palette import (
    DARKMODE_PALETTE_NAME,
    DEFAULT_PALETTE_NAME,
    EditorPalette,
)


def _color(red: int, green: int, blue: int) -> QColor:
    """Create a QColor from RGB values."""
    return QColor(red, green, blue)


PALETTES = {
    DEFAULT_PALETTE_NAME: EditorPalette(
        background=_color(255, 255, 255),
        text_primary=_color(18, 18, 18),
        text_secondary=_color(80, 80, 80),
        ui_window_bg=_color(246, 246, 246),
        ui_panel_bg=_color(236, 236, 236),
        ui_panel_alt_bg=_color(226, 226, 226),
        ui_input_bg=_color(255, 255, 255),
        ui_button_bg=_color(240, 240, 240),
        ui_button_hover_bg=_color(228, 228, 228),
        ui_button_pressed_bg=_color(214, 214, 214),
        ui_border=_color(176, 176, 176),
        ui_selection_bg=_color(65, 110, 185),
        ui_selection_text=_color(255, 255, 255),
        ui_tooltip_bg=_color(255, 255, 220),
        ui_tooltip_text=_color(20, 20, 20),
        state_python_fill=_color(144, 238, 144),
        state_cpp_fill=_color(255, 182, 193),
        state_xml_fill=_color(255, 165, 0),
        state_pen=_color(44, 77, 134),
        container_xml_fill=_color(255, 165, 0),
        container_xml_pen=_color(168, 104, 10),
        container_concurrence_fill=_color(255, 235, 205),
        container_concurrence_pen=_color(177, 120, 34),
        container_state_machine_fill=_color(220, 235, 255),
        container_state_machine_pen=_color(69, 106, 162),
        final_outcome_fill=_color(215, 55, 55),
        final_outcome_pen=_color(133, 28, 28),
        selection_pen=_color(255, 200, 0),
        blackboard_highlight_pen=_color(255, 170, 0),
        blackboard_highlight_fill=_color(255, 255, 170),
        runtime_highlight_pen=_color(46, 150, 76),
        runtime_highlight_fill=_color(198, 244, 210),
        runtime_transition_pen=_color(34, 122, 194),
        runtime_transition_label_bg=_color(220, 239, 255),
        runtime_canvas_border=_color(46, 150, 76),
        runtime_mode_button_bg=_color(46, 150, 76),
        runtime_mode_button_text=_color(255, 255, 255),
        runtime_log_default=_color(18, 18, 18),
        runtime_log_status=_color(80, 80, 80),
        runtime_log_info=_color(18, 18, 18),
        runtime_log_debug=_color(0, 128, 0),
        runtime_log_warn=_color(138, 103, 0),
        runtime_log_error=_color(181, 51, 51),
        runtime_log_system=_color(21, 101, 192),
        connection_line=_color(60, 60, 180),
        connection_selected=_color(255, 100, 0),
        connection_label_bg=_color(255, 255, 255),
        connection_label_pen=_color(60, 60, 180),
        connection_label_text=_color(0, 0, 100),
        connection_port_fill=_color(100, 100, 255),
        connection_port_pen=_color(44, 77, 134),
        temp_connection=_color(100, 100, 255),
    ),
    DARKMODE_PALETTE_NAME: EditorPalette(
        background=_color(33, 37, 43),
        text_primary=_color(232, 235, 239),
        text_secondary=_color(172, 178, 186),
        ui_window_bg=_color(35, 38, 43),
        ui_panel_bg=_color(43, 47, 54),
        ui_panel_alt_bg=_color(51, 56, 64),
        ui_input_bg=_color(28, 31, 36),
        ui_button_bg=_color(58, 64, 72),
        ui_button_hover_bg=_color(70, 76, 86),
        ui_button_pressed_bg=_color(82, 89, 100),
        ui_border=_color(78, 84, 94),
        ui_selection_bg=_color(74, 104, 155),
        ui_selection_text=_color(245, 247, 250),
        ui_tooltip_bg=_color(45, 49, 56),
        ui_tooltip_text=_color(235, 238, 242),
        state_python_fill=_color(79, 133, 96),
        state_cpp_fill=_color(144, 84, 97),
        state_xml_fill=_color(164, 113, 48),
        state_pen=_color(34, 57, 86),
        container_xml_fill=_color(146, 101, 41),
        container_xml_pen=_color(96, 64, 24),
        container_concurrence_fill=_color(95, 81, 55),
        container_concurrence_pen=_color(62, 53, 36),
        container_state_machine_fill=_color(67, 92, 120),
        container_state_machine_pen=_color(38, 56, 76),
        final_outcome_fill=_color(178, 60, 60),
        final_outcome_pen=_color(108, 33, 33),
        selection_pen=_color(255, 210, 90),
        blackboard_highlight_pen=_color(255, 196, 64),
        blackboard_highlight_fill=_color(125, 110, 42),
        runtime_highlight_pen=_color(116, 214, 146),
        runtime_highlight_fill=_color(44, 88, 56),
        runtime_transition_pen=_color(128, 192, 255),
        runtime_transition_label_bg=_color(42, 67, 92),
        runtime_canvas_border=_color(116, 214, 146),
        runtime_mode_button_bg=_color(68, 124, 84),
        runtime_mode_button_text=_color(245, 247, 250),
        runtime_log_default=_color(232, 234, 237),
        runtime_log_status=_color(211, 215, 220),
        runtime_log_info=_color(255, 255, 255),
        runtime_log_debug=_color(85, 255, 85),
        runtime_log_warn=_color(255, 210, 77),
        runtime_log_error=_color(255, 122, 122),
        runtime_log_system=_color(102, 179, 255),
        connection_line=_color(130, 170, 255),
        connection_selected=_color(255, 165, 90),
        connection_label_bg=_color(49, 55, 63),
        connection_label_pen=_color(130, 170, 255),
        connection_label_text=_color(235, 235, 235),
        connection_port_fill=_color(130, 170, 255),
        connection_port_pen=_color(34, 57, 86),
        temp_connection=_color(130, 170, 255),
    ),
}
