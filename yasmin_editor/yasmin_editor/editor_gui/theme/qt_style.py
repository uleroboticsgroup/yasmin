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


def _set_group_color(qt_palette: QPalette, role: QPalette.ColorRole, color) -> None:
    """Apply a color to all relevant palette groups."""
    for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
        qt_palette.setColor(group, role, color)


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


def build_qtconsole_palette(palette: EditorPalette) -> QPalette:
    """Build a dedicated Qt palette for the embedded interactive shell."""
    qt_palette = QPalette()

    _set_group_color(qt_palette, QPalette.Window, palette.shell_bg)
    _set_group_color(qt_palette, QPalette.WindowText, palette.shell_text)
    _set_group_color(qt_palette, QPalette.Base, palette.shell_bg)
    _set_group_color(qt_palette, QPalette.AlternateBase, palette.ui_panel_alt_bg)
    _set_group_color(qt_palette, QPalette.Text, palette.shell_text)
    _set_group_color(qt_palette, QPalette.Button, palette.ui_button_bg)
    _set_group_color(qt_palette, QPalette.ButtonText, palette.text_primary)
    _set_group_color(qt_palette, QPalette.ToolTipBase, palette.ui_tooltip_bg)
    _set_group_color(qt_palette, QPalette.ToolTipText, palette.ui_tooltip_text)
    _set_group_color(qt_palette, QPalette.Highlight, palette.shell_selection_bg)
    _set_group_color(qt_palette, QPalette.HighlightedText, palette.shell_selection_text)
    _set_group_color(qt_palette, QPalette.PlaceholderText, palette.text_secondary)
    _set_group_color(qt_palette, QPalette.Link, palette.shell_prompt_in)
    _set_group_color(qt_palette, QPalette.LinkVisited, palette.shell_prompt_out)
    _set_group_color(qt_palette, QPalette.Light, palette.ui_panel_alt_bg)
    _set_group_color(qt_palette, QPalette.Mid, palette.shell_border)
    _set_group_color(qt_palette, QPalette.Dark, palette.shell_border.darker(130))
    _set_group_color(qt_palette, QPalette.Shadow, palette.shell_border.darker(170))

    return qt_palette


def build_qtconsole_stylesheet(palette: EditorPalette) -> str:
    """Build a Qt stylesheet for the embedded interactive shell."""
    shell_bg = _css(palette.shell_bg)
    shell_text = _css(palette.shell_text)
    shell_border = _css(palette.shell_border)
    selection_bg = _css(palette.shell_selection_bg)
    selection_text = _css(palette.shell_selection_text)
    panel_bg = _css(palette.ui_panel_bg)
    panel_alt_bg = _css(palette.ui_panel_alt_bg)
    button_bg = _css(palette.ui_button_bg)
    button_hover_bg = _css(palette.ui_button_hover_bg)
    button_pressed_bg = _css(palette.ui_button_pressed_bg)
    tooltip_bg = _css(palette.ui_tooltip_bg)
    tooltip_text = _css(palette.ui_tooltip_text)
    prompt_in = _css(palette.shell_prompt_in)
    prompt_out = _css(palette.shell_prompt_out)
    comment = _css(palette.shell_comment)
    keyword = _css(palette.shell_keyword)
    string = _css(palette.shell_string)
    number = _css(palette.shell_number)
    error = _css(palette.shell_error)

    return f"""
QWidget {{
    background-color: {shell_bg};
    color: {shell_text};
    selection-background-color: {selection_bg};
    selection-color: {selection_text};
}}

QFrame,
QPlainTextEdit,
QTextEdit,
ConsoleWidget,
JupyterWidget,
RichJupyterWidget {{
    background-color: {shell_bg};
    color: {shell_text};
    selection-background-color: {selection_bg};
    selection-color: {selection_text};
    border: 1px solid {shell_border};
}}

QPlainTextEdit,
QTextEdit {{
    background-clip: padding;
}}

QAbstractScrollArea,
QAbstractItemView,
QListWidget,
QListView,
QTreeView {{
    background-color: {panel_bg};
    color: {shell_text};
    selection-background-color: {selection_bg};
    selection-color: {selection_text};
    border: 1px solid {shell_border};
    outline: 0;
    alternate-background-color: {panel_alt_bg};
    show-decoration-selected: 1;
}}

QAbstractItemView::item,
QListWidget::item,
QListView::item,
QTreeView::item {{
    background-color: transparent;
    color: {shell_text};
    padding: 2px 6px;
}}

QAbstractItemView::item:selected,
QAbstractItemView::item:selected:active,
QAbstractItemView::item:selected:!active,
QListWidget::item:selected,
QListWidget::item:selected:active,
QListWidget::item:selected:!active,
QListView::item:selected,
QListView::item:selected:active,
QListView::item:selected:!active,
QTreeView::item:selected,
QTreeView::item:selected:active,
QTreeView::item:selected:!active {{
    background-color: {selection_bg};
    color: {selection_text};
}}

QAbstractItemView::item:hover,
QListWidget::item:hover,
QListView::item:hover,
QTreeView::item:hover {{
    background-color: {button_hover_bg};
    color: {shell_text};
}}

QScrollBar,
QScrollBar::add-page,
QScrollBar::sub-page {{
    background-color: {shell_bg};
    color: {shell_text};
    border: 1px solid {shell_border};
}}

QScrollBar:vertical,
QScrollBar:horizontal {{
    background-color: {shell_bg};
    border: 1px solid {shell_border};
}}

QScrollBar::handle:vertical,
QScrollBar::handle:horizontal {{
    background-color: {button_bg};
    border: 1px solid {shell_border};
    min-height: 18px;
    min-width: 18px;
}}

QScrollBar::handle:vertical:hover,
QScrollBar::handle:horizontal:hover {{
    background-color: {button_hover_bg};
}}

QScrollBar::handle:vertical:pressed,
QScrollBar::handle:horizontal:pressed {{
    background-color: {button_pressed_bg};
}}

QToolTip {{
    background-color: {tooltip_bg};
    color: {tooltip_text};
    border: 1px solid {shell_border};
}}

.in-prompt,
.in-prompt-number {{
    color: {prompt_in};
    font-weight: bold;
}}

.out-prompt,
.out-prompt-number {{
    color: {prompt_out};
    font-weight: bold;
}}

.error,
.traceback {{
    color: {error};
}}

.inverted {{
    background-color: {shell_text};
    color: {shell_bg};
}}

.c,
.c1,
.cm {{
    color: {comment};
    font-style: italic;
}}

.k,
.kc,
.kd,
.kn,
.kp,
.kr,
.kt,
.o {{
    color: {keyword};
    font-weight: bold;
}}

.s,
.sa,
.sb,
.sc,
.sd,
.s2,
.se,
.sh,
.si,
.sr,
.ss,
.sx {{
    color: {string};
}}

.m,
.mb,
.mf,
.mh,
.mi,
.mo,
.il {{
    color: {number};
}}

.nb,
.bp {{
    color: {prompt_in};
}}

.ne,
.gr {{
    color: {error};
    font-weight: bold;
}}
"""


def build_qtconsole_syntax_style(palette: EditorPalette):
    """Build a Pygments syntax style class for the embedded interactive shell."""
    try:
        from pygments.style import Style
        from pygments.token import (
            Comment,
            Error,
            Generic,
            Keyword,
            Name,
            Number,
            Operator,
            String,
            Text,
        )
    except Exception:
        return None

    text = palette.shell_text.name()
    comment = palette.shell_comment.name()
    keyword = palette.shell_keyword.name()
    string = palette.shell_string.name()
    number = palette.shell_number.name()
    prompt_in = palette.shell_prompt_in.name()
    prompt_out = palette.shell_prompt_out.name()
    error = palette.shell_error.name()
    background = palette.shell_bg.name()
    highlight = palette.shell_selection_bg.name()

    class YASMINQtConsoleStyle(Style):
        background_color = background
        highlight_color = highlight
        default_style = ""
        styles = {
            Text: text,
            Comment: f"italic {comment}",
            Keyword: f"bold {keyword}",
            Operator: keyword,
            Name: text,
            Name.Builtin: prompt_in,
            Name.Class: text,
            Name.Function: text,
            Name.Exception: error,
            Number: number,
            String: string,
            Generic.Prompt: f"bold {prompt_in}",
            Generic.Output: prompt_out,
            Generic.Error: f"bold {error}",
            Error: f"bold {error}",
        }

    return YASMINQtConsoleStyle


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

QStatusBar,
QMenuBar,
QMenu,
QFrame,
QSplitter::handle {{
    background-color: {panel_bg};
    color: {text_primary};
    border-color: {border};
}}

QToolBar#yasminEditorToolbar {{
    background-color: {panel_bg};
    color: {text_primary};
    border-color: {border};
    border-bottom: 1px solid {border};
    spacing: 0px;
    padding: 0px;
}}

QToolBar#yasminEditorToolbar::separator {{
    background: transparent;
    width: 1px;
    margin: 6px 6px;
    border-left: 1px solid {border};
}}

QToolBar#yasminEditorToolbar QToolButton {{
    background-color: transparent;
    color: {text_primary};
    border: none;
    padding: 4px 8px;
    margin: 0px;
}}

QToolBar#yasminEditorToolbar QToolButton:hover {{
    background-color: {button_hover_bg};
}}

QToolBar#yasminEditorToolbar QToolButton:pressed,
QToolBar#yasminEditorToolbar QToolButton:checked {{
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
    min-height: 20px;
    min-width: 20px;
}}

QHeaderView::section {{
    background-color: {panel_alt_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 2px 4px;
}}

QGroupBox {{
    border: 1px solid {border};
    margin-top: 6px;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}}

QToolTip {{
    background-color: {tooltip_bg};
    color: {tooltip_text};
    border: 1px solid {border};
}}

QTabWidget::pane,
QDockWidget::title {{
    background-color: {panel_bg};
    border: 1px solid {border};
}}

QTabBar::tab {{
    background-color: {panel_alt_bg};
    color: {text_primary};
    border: 1px solid {border};
    padding: 4px 8px;
}}

QTabBar::tab:selected {{
    background-color: {panel_bg};
}}

QTreeView::branch:selected,
QListView::item:selected,
QTreeWidget::item:selected,
QTableWidget::item:selected {{
    background-color: {selection_bg};
    color: {selection_text};
}}

QWidget:disabled,
QLabel:disabled,
QPushButton:disabled,
QLineEdit:disabled,
QTextEdit:disabled,
QPlainTextEdit:disabled,
QComboBox:disabled,
QCheckBox:disabled,
QRadioButton:disabled,
QTableWidget:disabled,
QTreeWidget:disabled,
QListWidget:disabled {{
    color: {text_secondary};
}}
"""
