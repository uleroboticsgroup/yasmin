#!/usr/bin/env python3
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
"""Compatibility exports for selection bundle collection, paste, and removal."""

from yasmin_editor.editor_gui.selection_bundle_collect import collect_selection_bundle
from yasmin_editor.editor_gui.selection_bundle_geometry import get_bundle_bounds
from yasmin_editor.editor_gui.selection_bundle_paste import paste_bundle_into_model
from yasmin_editor.editor_gui.selection_bundle_remove import remove_selection_from_model

__all__ = [
    "collect_selection_bundle",
    "get_bundle_bounds",
    "paste_bundle_into_model",
    "remove_selection_from_model",
]
