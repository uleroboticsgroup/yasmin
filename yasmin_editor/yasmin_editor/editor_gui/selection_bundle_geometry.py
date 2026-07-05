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

from typing import Iterable, Tuple
from yasmin_editor.editor_gui.selection_models import SelectionBundle


def iter_bundle_points(bundle: SelectionBundle) -> Iterable[Tuple[float, float]]:
    """Yield all positions that participate in the selection bounds."""

    for position in bundle.state_positions.values():
        yield (position.x, position.y)
    for placement in bundle.outcome_placements:
        yield (placement.position.x, placement.position.y)
    for text_block in bundle.text_blocks:
        yield (text_block.x, text_block.y)


def get_bundle_bounds(bundle: SelectionBundle) -> Tuple[float, float, float, float]:
    """Return the selection bounding box as min_x, min_y, max_x, max_y."""

    points = list(iter_bundle_points(bundle))
    if not points:
        return (0.0, 0.0, 0.0, 0.0)

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs), min(ys), max(xs), max(ys))
