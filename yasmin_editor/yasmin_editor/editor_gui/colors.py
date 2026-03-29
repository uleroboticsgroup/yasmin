# Copyright (C) 2025 Miguel Ángel González Santamarta
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

import os
from dataclasses import dataclass
from typing import Optional

from PyQt5.QtGui import QColor, QPalette


@dataclass(frozen=True)
class EditorPalette:
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
    connection_line: QColor
    connection_selected: QColor
    connection_label_bg: QColor
    connection_label_pen: QColor
    connection_label_text: QColor
    connection_port_fill: QColor
    connection_port_pen: QColor
    temp_connection: QColor

    def state_fill(self, plugin_type: Optional[str]) -> QColor:
        if plugin_type == "python":
            return self.state_python_fill
        if plugin_type == "cpp":
            return self.state_cpp_fill
        return self.state_xml_fill


def _css(color: QColor) -> str:
    return color.name()


def build_qt_palette(palette: EditorPalette) -> QPalette:
    qt_palette = QPalette()
    qt_palette.setColor(QPalette.Window, palette.ui_window_bg)
    qt_palette.setColor(QPalette.WindowText, palette.text_primary)
    qt_palette.setColor(QPalette.Base, palette.ui_input_bg)
    qt_palette.setColor(QPalette.AlternateBase, palette.ui_panel_alt_bg)
    qt_palette.setColor(QPalette.ToolTipBase, palette.ui_tooltip_bg)
    qt_palette.setColor(QPalette.ToolTipText, palette.ui_tooltip_text)
    qt_palette.setColor(QPalette.Text, palette.text_primary)
    qt_palette.setColor(QPalette.Button, palette.ui_button_bg)
    qt_palette.setColor(QPalette.ButtonText, palette.text_primary)
    qt_palette.setColor(QPalette.BrightText, palette.ui_selection_text)
    qt_palette.setColor(QPalette.Highlight, palette.ui_selection_bg)
    qt_palette.setColor(QPalette.HighlightedText, palette.ui_selection_text)
    qt_palette.setColor(QPalette.Light, palette.ui_panel_alt_bg)
    qt_palette.setColor(QPalette.Midlight, palette.ui_panel_bg)
    qt_palette.setColor(QPalette.Mid, palette.ui_border)
    qt_palette.setColor(QPalette.Dark, palette.ui_border.darker(130))
    qt_palette.setColor(QPalette.Shadow, palette.ui_border.darker(170))
    qt_palette.setColor(QPalette.Link, palette.connection_line)
    qt_palette.setColor(QPalette.LinkVisited, palette.connection_selected)
    qt_palette.setColor(QPalette.PlaceholderText, palette.text_secondary)
    return qt_palette


def build_stylesheet(palette: EditorPalette) -> str:
    window_bg = _css(palette.ui_window_bg)
    panel_bg = _css(palette.ui_panel_bg)
    panel_alt_bg = _css(palette.ui_panel_alt_bg)
    input_bg = _css(palette.ui_input_bg)
    button_bg = _css(palette.ui_button_bg)
    button_hover_bg = _css(palette.ui_button_hover_bg)
    button_pressed_bg = _css(palette.ui_button_pressed_bg)
    border = _css(palette.ui_border)
    text_primary = _css(palette.text_primary)
    text_secondary = _css(palette.text_secondary)
    selection_bg = _css(palette.ui_selection_bg)
    selection_text = _css(palette.ui_selection_text)
    tooltip_bg = _css(palette.ui_tooltip_bg)
    tooltip_text = _css(palette.ui_tooltip_text)

    return f"""
QMainWindow,
QDialog {{
    background-color: {panel_bg};
    color: {text_primary};
}}

QWidget {{
    color: {text_primary};
}}

QLabel {{
    background-color: transparent;
    color: {text_primary};
}}

QToolBar,
QStatusBar,
QMenuBar,
QMenu,
QFrame,
QSplitter::handle {{
    background-color: {panel_bg};
    color: {text_primary};
    border-color: {border};
}}

QToolBar {{
    border-bottom: 1px solid {border};
    spacing: 0px;
    padding: 0px;
}}

QToolBar::separator {{
    background: transparent;
    width: 1px;
    margin: 6px 6px;
    border-left: 1px solid {border};
}}

QToolButton {{
    background-color: transparent;
    color: {text_primary};
    border: none;
    padding: 4px 8px;
    margin: 0px;
}}

QToolButton:hover {{
    background-color: {button_hover_bg};
}}

QToolButton:pressed,
QToolButton:checked {{
    background-color: {button_pressed_bg};
}}

QStatusBar {{
    border-top: 1px solid {border};
}}

QMenuBar::item,
QMenu::item {{
    background: transparent;
    color: {text_primary};
}}

QMenuBar::item:selected,
QMenu::item:selected {{
    background-color: {button_hover_bg};
}}

QLineEdit,
QTextEdit,
QPlainTextEdit,
QTextBrowser,
QListWidget,
QTreeWidget,
QTableWidget,
QComboBox,
QAbstractSpinBox {{
    background-color: {input_bg};
    color: {text_primary};
    border: 1px solid {border};
    selection-background-color: {selection_bg};
    selection-color: {selection_text};
}}

QLineEdit[flatInput="true"],
QComboBox[flatInput="true"] {{
    background-color: {window_bg};
    border: 1px solid transparent;
    border-bottom: 1px solid {border};
    border-radius: 0px;
    padding: 2px 4px;
}}

QComboBox[flatInput="true"] {{
    padding-right: 18px;
}}

QLineEdit:focus,
QTextEdit:focus,
QPlainTextEdit:focus,
QTextBrowser:focus,
QListWidget:focus,
QTreeWidget:focus,
QTableWidget:focus,
QComboBox:focus,
QAbstractSpinBox:focus {{
    border: 1px solid {selection_bg};
}}

QLineEdit[flatInput="true"]:focus,
QComboBox[flatInput="true"]:focus {{
    border: 1px solid transparent;
    border-bottom: 1px solid {selection_bg};
}}

QComboBox::drop-down {{
    background-color: {button_bg};
    border-left: 1px solid {border};
}}

QComboBox[flatInput="true"]::drop-down {{
    background-color: transparent;
    border: none;
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 16px;
}}

QAbstractItemView {{
    background-color: {input_bg};
    color: {text_primary};
    selection-background-color: {selection_bg};
    selection-color: {selection_text};
    border: 1px solid {border};
}}

QPushButton {{
    background-color: {button_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 4px 10px;
}}

QPushButton:hover {{
    background-color: {button_hover_bg};
}}

QPushButton:pressed,
QPushButton:checked {{
    background-color: {button_pressed_bg};
}}

QPushButton[flat="true"] {{
    background-color: transparent;
    border: 1px solid transparent;
    padding: 2px 6px;
}}

QPushButton[flat="true"]:hover {{
    background-color: {button_hover_bg};
    border: 1px solid {border};
}}

QTextEdit[viewerText="true"],
QTextBrowser[viewerText="true"] {{
    background-color: {window_bg};
    color: {text_primary};
    border: 1px solid {border};
}}

QLabel[infoBox="true"] {{
    background-color: {window_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 4px;
}}

QScrollBar:vertical,
QScrollBar:horizontal {{
    background-color: {panel_bg};
    border: 1px solid {border};
}}

QScrollBar::handle:vertical,
QScrollBar::handle:horizontal {{
    background-color: {button_bg};
    min-height: 24px;
    min-width: 24px;
}}

QScrollBar::handle:vertical:hover,
QScrollBar::handle:horizontal:hover {{
    background-color: {button_hover_bg};
}}

QScrollBar::add-line,
QScrollBar::sub-line,
QScrollBar::add-page,
QScrollBar::sub-page {{
    background: {panel_alt_bg};
    border: none;
}}

QToolTip {{
    background-color: {tooltip_bg};
    color: {tooltip_text};
    border: 1px solid {border};
}}

QGraphicsView {{
    background-color: {_css(palette.background)};
    border: 1px solid {border};
}}

QListWidget::item,
QTreeWidget::item,
QTableWidget::item {{
    color: {text_primary};
}}

QListWidget::item:selected,
QTreeWidget::item:selected,
QTableWidget::item:selected {{
    background-color: {selection_bg};
    color: {selection_text};
}}

QHeaderView::section {{
    background-color: {panel_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 4px;
}}

QTabWidget::pane {{
    border: 1px solid {border};
}}

QTabBar::tab {{
    background-color: {panel_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 4px 10px;
}}

QTabBar::tab:selected {{
    background-color: {button_hover_bg};
}}

QCheckBox,
QRadioButton,
QGroupBox {{
    color: {text_primary};
}}

QGroupBox {{
    border: 1px solid {border};
    margin-top: 8px;
    padding-top: 8px;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: {text_secondary};
}}
""".strip()


PALETTES = {
    "default": EditorPalette(
        background=QColor(255, 255, 255),
        text_primary=QColor(18, 18, 18),
        text_secondary=QColor(80, 80, 80),
        ui_window_bg=QColor(246, 246, 246),
        ui_panel_bg=QColor(236, 236, 236),
        ui_panel_alt_bg=QColor(226, 226, 226),
        ui_input_bg=QColor(255, 255, 255),
        ui_button_bg=QColor(240, 240, 240),
        ui_button_hover_bg=QColor(228, 228, 228),
        ui_button_pressed_bg=QColor(214, 214, 214),
        ui_border=QColor(176, 176, 176),
        ui_selection_bg=QColor(65, 110, 185),
        ui_selection_text=QColor(255, 255, 255),
        ui_tooltip_bg=QColor(255, 255, 220),
        ui_tooltip_text=QColor(20, 20, 20),
        state_python_fill=QColor(144, 238, 144),
        state_cpp_fill=QColor(255, 182, 193),
        state_xml_fill=QColor(255, 165, 0),
        state_pen=QColor(44, 77, 134),
        container_xml_fill=QColor(255, 165, 0),
        container_xml_pen=QColor(168, 104, 10),
        container_concurrence_fill=QColor(255, 235, 205),
        container_concurrence_pen=QColor(177, 120, 34),
        container_state_machine_fill=QColor(220, 235, 255),
        container_state_machine_pen=QColor(69, 106, 162),
        final_outcome_fill=QColor(215, 55, 55),
        final_outcome_pen=QColor(133, 28, 28),
        selection_pen=QColor(255, 200, 0),
        blackboard_highlight_pen=QColor(255, 170, 0),
        blackboard_highlight_fill=QColor(255, 255, 170),
        runtime_highlight_pen=QColor(46, 150, 76),
        runtime_highlight_fill=QColor(198, 244, 210),
        runtime_transition_pen=QColor(34, 122, 194),
        runtime_transition_label_bg=QColor(220, 239, 255),
        runtime_canvas_border=QColor(46, 150, 76),
        runtime_mode_button_bg=QColor(46, 150, 76),
        runtime_mode_button_text=QColor(255, 255, 255),
        connection_line=QColor(60, 60, 180),
        connection_selected=QColor(255, 100, 0),
        connection_label_bg=QColor(255, 255, 255),
        connection_label_pen=QColor(60, 60, 180),
        connection_label_text=QColor(0, 0, 100),
        connection_port_fill=QColor(100, 100, 255),
        connection_port_pen=QColor(44, 77, 134),
        temp_connection=QColor(100, 100, 255),
    ),
    "darkmode": EditorPalette(
        background=QColor(33, 37, 43),
        text_primary=QColor(232, 235, 239),
        text_secondary=QColor(172, 178, 186),
        ui_window_bg=QColor(35, 38, 43),
        ui_panel_bg=QColor(43, 47, 54),
        ui_panel_alt_bg=QColor(51, 56, 64),
        ui_input_bg=QColor(28, 31, 36),
        ui_button_bg=QColor(58, 64, 72),
        ui_button_hover_bg=QColor(70, 76, 86),
        ui_button_pressed_bg=QColor(82, 89, 100),
        ui_border=QColor(78, 84, 94),
        ui_selection_bg=QColor(74, 104, 155),
        ui_selection_text=QColor(245, 247, 250),
        ui_tooltip_bg=QColor(45, 49, 56),
        ui_tooltip_text=QColor(235, 238, 242),
        state_python_fill=QColor(79, 133, 96),
        state_cpp_fill=QColor(144, 84, 97),
        state_xml_fill=QColor(164, 113, 48),
        state_pen=QColor(34, 57, 86),
        container_xml_fill=QColor(146, 101, 41),
        container_xml_pen=QColor(96, 64, 24),
        container_concurrence_fill=QColor(95, 81, 55),
        container_concurrence_pen=QColor(62, 53, 36),
        container_state_machine_fill=QColor(67, 92, 120),
        container_state_machine_pen=QColor(38, 56, 76),
        final_outcome_fill=QColor(178, 60, 60),
        final_outcome_pen=QColor(108, 33, 33),
        selection_pen=QColor(255, 210, 90),
        blackboard_highlight_pen=QColor(255, 196, 64),
        blackboard_highlight_fill=QColor(125, 110, 42),
        runtime_highlight_pen=QColor(116, 214, 146),
        runtime_highlight_fill=QColor(44, 88, 56),
        runtime_transition_pen=QColor(128, 192, 255),
        runtime_transition_label_bg=QColor(42, 67, 92),
        runtime_canvas_border=QColor(116, 214, 146),
        runtime_mode_button_bg=QColor(68, 124, 84),
        runtime_mode_button_text=QColor(245, 247, 250),
        connection_line=QColor(130, 170, 255),
        connection_selected=QColor(255, 165, 90),
        connection_label_bg=QColor(49, 55, 63),
        connection_label_pen=QColor(130, 170, 255),
        connection_label_text=QColor(235, 235, 235),
        connection_port_fill=QColor(130, 170, 255),
        connection_port_pen=QColor(34, 57, 86),
        temp_connection=QColor(130, 170, 255),
    ),
}


def _normalize_palette_name(value: str) -> str:
    normalized = value.strip().lower().replace("-", "").replace("_", "")
    if normalized in {"dark", "darkmode"}:
        return "darkmode"
    return "default"


PALETTE_NAME = _normalize_palette_name(os.getenv("YASMIN_EDITOR_COLOR", "default"))
PALETTE = PALETTES[PALETTE_NAME]
