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

"""Qt-free helpers for manual selection placement previews."""

from __future__ import annotations

from dataclasses import dataclass

from yasmin_editor.editor_gui.selection_bundle_geometry import get_bundle_bounds
from yasmin_editor.editor_gui.selection_models import SelectionBundle


@dataclass(frozen=True, slots=True)
class SelectionPlacementPreview:
    """Lightweight preview metadata shown while a copied bundle follows the cursor."""

    label: str
    width: float
    height: float


_MIN_PREVIEW_WIDTH = 120.0
_MIN_PREVIEW_HEIGHT = 72.0


def _bundle_item_summary(bundle: SelectionBundle) -> str:
    parts: list[str] = []
    if bundle.states:
        parts.append(
            f"{len(bundle.states)} state{'s' if len(bundle.states) != 1 else ''}"
        )
    if bundle.outcome_placements:
        parts.append(
            f"{len(bundle.outcome_placements)} outcome{'s' if len(bundle.outcome_placements) != 1 else ''}"
        )
    if bundle.text_blocks:
        parts.append(
            f"{len(bundle.text_blocks)} note{'s' if len(bundle.text_blocks) != 1 else ''}"
        )
    return ", ".join(parts) if parts else "selection"


def build_selection_preview(
    bundle: SelectionBundle, *, verb: str
) -> SelectionPlacementPreview:
    """Return a stable preview rectangle and label for one pending bundle placement."""

    min_x, min_y, max_x, max_y = get_bundle_bounds(bundle)
    width = max(_MIN_PREVIEW_WIDTH, float(max_x - min_x))
    height = max(_MIN_PREVIEW_HEIGHT, float(max_y - min_y))
    label = f"{verb} {_bundle_item_summary(bundle)}"
    return SelectionPlacementPreview(label=label, width=width, height=height)
