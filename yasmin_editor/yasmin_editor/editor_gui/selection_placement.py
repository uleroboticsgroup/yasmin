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

from __future__ import annotations

from typing import List
from yasmin_editor.dataclass_compat import dataclass
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
    parts: List[str] = []
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
