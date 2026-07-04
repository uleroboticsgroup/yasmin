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

from yasmin_editor.editor_gui.theme.palette import (
    DARKMODE_PALETTE_NAME,
    DEFAULT_PALETTE_NAME,
    PALETTE_NAME,
    YASMIN_EDITOR_THEME_ENV,
    EditorPalette,
    get_palette_name_from_env,
    normalize_palette_name,
)
from yasmin_editor.editor_gui.theme.palette_data import PALETTES
from yasmin_editor.editor_gui.theme.qt_style import build_qt_palette, build_stylesheet

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
