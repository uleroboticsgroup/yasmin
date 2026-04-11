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
"""Pure helper for selecting a free scene position near the viewport center."""

from __future__ import annotations

PointTuple = tuple[float, float]


def find_free_position(
    center: PointTuple,
    occupied_positions: list[PointTuple],
    *,
    spacing_x: float = 180.0,
    spacing_y: float = 130.0,
    radius_limit: int = 5,
) -> PointTuple:
    """Return a free grid slot near the visible center.

    Candidates are scanned in expanding square rings around the viewport center.
    When no free slot is found inside the search radius, the helper falls back to
    a deterministic overflow grid so newly created items still land in a stable
    location.
    """

    for candidate in iter_candidate_positions(
        center,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        radius_limit=radius_limit,
    ):
        if is_position_free(
            candidate,
            occupied_positions,
            spacing_x=spacing_x,
            spacing_y=spacing_y,
        ):
            return candidate
    return fallback_position(
        center,
        len(occupied_positions),
        spacing_x=spacing_x,
        spacing_y=spacing_y,
    )


def iter_candidate_positions(
    center: PointTuple,
    *,
    spacing_x: float,
    spacing_y: float,
    radius_limit: int,
) -> list[PointTuple]:
    """Return candidate positions in the same ring order used by the editor."""

    center_x, center_y = center
    candidates: list[PointTuple] = [(center_x, center_y)]
    for radius in range(1, radius_limit + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                candidates.append((center_x + dx * spacing_x, center_y + dy * spacing_y))
    return candidates


def is_position_free(
    candidate: PointTuple,
    occupied_positions: list[PointTuple],
    *,
    spacing_x: float,
    spacing_y: float,
) -> bool:
    """Return whether one candidate is far enough away from existing items."""

    for occupied_x, occupied_y in occupied_positions:
        if (
            abs(candidate[0] - occupied_x) < spacing_x * 0.8
            and abs(candidate[1] - occupied_y) < spacing_y * 0.8
        ):
            return False
    return True


def fallback_position(
    center: PointTuple,
    occupied_count: int,
    *,
    spacing_x: float,
    spacing_y: float,
) -> PointTuple:
    """Return a deterministic overflow slot when the search ring is saturated."""

    return (
        center[0] + (occupied_count % 4) * spacing_x,
        center[1] + (occupied_count // 4) * spacing_y,
    )
