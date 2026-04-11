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

"""Runtime presentation helpers shared by the editor runtime UI.

The runtime mixin contains the window-orchestration code, but the color and
formatting rules used by the runtime badge, runtime log, and runtime buttons
are easier to maintain as small pure helpers. Keeping these rules in one module
also makes them directly unit-testable without driving a live editor window.
"""

from __future__ import annotations

from dataclasses import dataclass
from html import escape

_RUNTIME_LOG_PREFIX_COLORS = {
    "[WARN]": "runtime_log_warn",
    "[ERROR]": "runtime_log_error",
    "[STATUS]": "runtime_log_status",
    "[INFO]": "runtime_log_info",
    "[DEBUG]": "runtime_log_debug",
    "[TRANSITION]": "runtime_log_system",
    "[START]": "runtime_log_system",
    "[END]": "runtime_log_system",
}

try:
    from PyQt5.QtGui import QColor as _QtQColor
except ModuleNotFoundError:
    _QtQColor = None

try:
    from yasmin_editor.editor_gui.colors import PALETTE as _ACTIVE_PALETTE
except Exception:
    _ACTIVE_PALETTE = None


class ColorToken(str):
    """Tiny color wrapper that mirrors the `QColor.name()` API used in tests."""

    def name(self) -> str:
        return str(self)


@dataclass(frozen=True)
class RuntimePaletteTokens:
    """Palette subset needed by the runtime UI helpers."""

    text_primary: str
    ui_input_bg: str
    ui_button_bg: str
    ui_border: str
    ui_selection_bg: str
    ui_selection_text: str
    runtime_mode_button_bg: str
    runtime_mode_button_text: str
    runtime_canvas_border: str
    runtime_log_default: str
    runtime_log_status: str
    runtime_log_info: str
    runtime_log_debug: str
    runtime_log_warn: str
    runtime_log_error: str
    runtime_log_system: str


_FALLBACK_TOKENS = RuntimePaletteTokens(
    text_primary="#121212",
    ui_input_bg="#ffffff",
    ui_button_bg="#f0f0f0",
    ui_border="#b0b0b0",
    ui_selection_bg="#416eb9",
    ui_selection_text="#ffffff",
    runtime_mode_button_bg="#2e964c",
    runtime_mode_button_text="#ffffff",
    runtime_canvas_border="#2e964c",
    runtime_log_default="#121212",
    runtime_log_status="#505050",
    runtime_log_info="#121212",
    runtime_log_debug="#008000",
    runtime_log_warn="#8a6700",
    runtime_log_error="#b53333",
    runtime_log_system="#1565c0",
)


def _color_token(value: str):
    """Return a color object that supports `.name()` with or without Qt."""
    if _QtQColor is not None:
        return _QtQColor(value)
    return ColorToken(value)


def _palette_value(name: str) -> str:
    """Resolve one palette color to a hex string, with a Qt-free fallback."""
    if _ACTIVE_PALETTE is None:
        return getattr(_FALLBACK_TOKENS, name)
    return getattr(_ACTIVE_PALETTE, name).name()


def runtime_status_badge_colors(status: str):
    """Return background, foreground, and border colors for a status badge."""
    normalized = str(status).strip().lower()

    background = _color_token("#cdd2d6")
    foreground = _color_token("#1c1f24")
    border = _color_token("#949aa0")

    if normalized == "running":
        background = _color_token("#2e964c")
        foreground = _color_token("#ffffff")
        border = _color_token("#216c36")
    elif normalized == "paused":
        background = _color_token("#d6b044")
        foreground = _color_token("#1e2228")
        border = _color_token("#ac882c")
    elif normalized in {"blocked", "waiting", "idle"}:
        background = _color_token("#6991c9")
        foreground = _color_token("#ffffff")
        border = _color_token("#47689c")
    elif normalized in {"ready", "inactive"}:
        background = _color_token("#cdd2d6")
        foreground = _color_token("#1e2228")
        border = _color_token("#acb2b8")
    elif normalized in {"succeeded", "success"}:
        background = _color_token("#2e964c")
        foreground = _color_token("#ffffff")
        border = _color_token("#216c36")
    elif normalized in {"aborted", "failed", "failure", "error"}:
        background = _color_token("#b03c3c")
        foreground = _color_token("#ffffff")
        border = _color_token("#782828")

    return background, foreground, border


def runtime_status_badge_style(status: str) -> str:
    """Return the stylesheet used by the runtime status badge."""
    background, foreground, border = runtime_status_badge_colors(status)
    return (
        "QLabel {"
        f"background-color: {background.name()}; "
        f"color: {foreground.name()}; "
        f"border: 1px solid {border.name()}; "
        "border-radius: 10px; "
        "padding: 4px 12px; "
        "font-weight: 600;"
        "}"
    )


def runtime_log_view_style() -> str:
    """Return the stylesheet used by the runtime log widget."""
    return (
        "QTextBrowser {"
        f"background-color: {_palette_value('ui_input_bg')}; "
        f"color: {_palette_value('text_primary')}; "
        f"border: 1px solid {_palette_value('ui_border')}; "
        f"selection-background-color: {_palette_value('ui_selection_bg')}; "
        f"selection-color: {_palette_value('ui_selection_text')}; "
        "border-radius: 6px;"
        "}"
    )


def _normalized_runtime_log_content(message: str) -> str:
    """Return the content portion of one runtime log entry starting at its first prefix."""

    raw_message = str(message)
    first_bracket_index = raw_message.find("[")
    if first_bracket_index < 0:
        return raw_message
    return raw_message[first_bracket_index:].lstrip()


def runtime_log_line_color(message: str) -> str:
    """Return the configured HTML color for one runtime log line."""

    normalized = _normalized_runtime_log_content(message)
    for prefix, token_name in _RUNTIME_LOG_PREFIX_COLORS.items():
        if normalized.startswith(prefix):
            return _palette_value(token_name)
    return _palette_value("runtime_log_default")


def format_runtime_log_entry(message: str) -> str:
    """Convert a runtime log message into the styled HTML fragment shown in the UI."""
    raw_message = str(message)
    color = runtime_log_line_color(raw_message)

    prefix = ""
    content = raw_message
    first_bracket_index = raw_message.find("[")
    if first_bracket_index > 0:
        prefix = raw_message[:first_bracket_index]
        content = _normalized_runtime_log_content(raw_message)
    elif first_bracket_index == 0:
        content = _normalized_runtime_log_content(raw_message)

    return (
        '<div style="'
        "font-family: 'DejaVu Sans Mono', 'Courier New', monospace; "
        "white-space: pre; "
        'margin: 0;">'
        f'<span style="color: {_palette_value("text_primary")};">{escape(prefix)}</span>'
        f'<span style="color: {color};">{escape(content)}</span>'
        "</div>"
    )


def runtime_toggle_button_style(checked: bool) -> str:
    """Return the stylesheet for the auto-follow style toggle buttons."""
    if checked:
        return (
            "QPushButton {"
            f"background-color: {_palette_value('runtime_mode_button_bg')}; "
            f"color: {_palette_value('runtime_mode_button_text')}; "
            f"border: 2px solid {_palette_value('runtime_canvas_border')}; "
            "font-weight: 700;"
            "padding: 4px 10px;"
            "}"
        )

    return (
        "QPushButton {"
        f"background-color: {_palette_value('ui_button_bg')}; "
        f"color: {_palette_value('text_primary')}; "
        f"border: 2px solid {_palette_value('ui_border')}; "
        "font-weight: 700;"
        "padding: 4px 10px;"
        "}"
    )


def runtime_mode_button_style(active: bool) -> str:
    """Return the stylesheet for the main runtime-mode toolbar button."""
    if not active:
        return ""

    return (
        "QPushButton {"
        f"background-color: {_palette_value('runtime_mode_button_bg')}; "
        f"color: {_palette_value('runtime_mode_button_text')}; "
        f"border: 1px solid {_palette_value('runtime_canvas_border')};"
        "}"
    )


def runtime_canvas_frame_style(border_width: int, border_color) -> str:
    """Return the stylesheet for the canvas frame shown around the editor scene."""
    color_name = (
        border_color.name() if hasattr(border_color, "name") else str(border_color)
    )
    return f"QFrame {{ border: {int(border_width)}px solid {color_name}; }}"
