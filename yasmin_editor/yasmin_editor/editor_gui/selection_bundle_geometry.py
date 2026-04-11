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
"""Geometry helpers for clipboard bundles."""

from __future__ import annotations

from typing import Iterable

from yasmin_editor.editor_gui.selection_models import SelectionBundle


def iter_bundle_points(bundle: SelectionBundle) -> Iterable[tuple[float, float]]:
    """Yield all positions that participate in the selection bounds."""

    for position in bundle.state_positions.values():
        yield (position.x, position.y)
    for placement in bundle.outcome_placements:
        yield (placement.position.x, placement.position.y)
    for text_block in bundle.text_blocks:
        yield (text_block.x, text_block.y)


def get_bundle_bounds(bundle: SelectionBundle) -> tuple[float, float, float, float]:
    """Return the selection bounding box as min_x, min_y, max_x, max_y."""

    points = list(iter_bundle_points(bundle))
    if not points:
        return (0.0, 0.0, 0.0, 0.0)

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs), min(ys), max(xs), max(ys))
