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

"""Compatibility layer for editor theme access.

The theme implementation was split into dedicated modules to keep palette data,
palette selection and Qt stylesheet generation separate. Existing imports keep
working through this wrapper.
"""

from yasmin_editor.editor_gui.theme import (
    DARKMODE_PALETTE_NAME,
    DEFAULT_PALETTE_NAME,
    PALETTE,
    PALETTE_NAME,
    PALETTES,
    YASMIN_EDITOR_THEME_ENV,
    EditorPalette,
    build_qt_palette,
    build_stylesheet,
    get_palette_name_from_env,
    normalize_palette_name,
)

__all__ = [
    "DARKMODE_PALETTE_NAME",
    "DEFAULT_PALETTE_NAME",
    "EditorPalette",
    "PALETTE",
    "PALETTES",
    "PALETTE_NAME",
    "YASMIN_EDITOR_THEME_ENV",
    "build_qt_palette",
    "build_stylesheet",
    "get_palette_name_from_env",
    "normalize_palette_name",
]
