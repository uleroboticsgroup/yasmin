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

"""Qt palette and stylesheet builders for the editor theme."""

from PyQt5.QtGui import QPalette

from yasmin_editor.editor_gui.theme.palette import EditorPalette


def _css(color) -> str:
    """Convert a QColor into a CSS color string."""
    return color.name()


def build_qt_palette(palette: EditorPalette) -> QPalette:
    """Build the Qt application palette for the given editor palette."""
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
    """Build the application stylesheet for the given editor palette."""
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
