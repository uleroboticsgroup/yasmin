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

"""Public theme API for editor colors, palette selection and stylesheet generation."""

from yasmin_editor.editor_gui.theme.palette import (DARKMODE_PALETTE_NAME,
                                                    DEFAULT_PALETTE_NAME,
                                                    PALETTE_NAME,
                                                    YASMIN_EDITOR_THEME_ENV,
                                                    EditorPalette,
                                                    get_palette_name_from_env,
                                                    normalize_palette_name)
from yasmin_editor.editor_gui.theme.palette_data import PALETTES
from yasmin_editor.editor_gui.theme.qt_style import (build_qt_palette,
                                                     build_stylesheet)

PALETTE = PALETTES[PALETTE_NAME]

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
