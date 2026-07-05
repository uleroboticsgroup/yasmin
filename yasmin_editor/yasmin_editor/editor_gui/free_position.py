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

from typing import Tuple, List

PointTuple = Tuple[float, float]


def find_free_position(
    center: PointTuple,
    occupied_positions: List[PointTuple],
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
) -> List[PointTuple]:
    """Return candidate positions in the same ring order used by the editor."""

    center_x, center_y = center
    candidates: List[PointTuple] = [(center_x, center_y)]
    for radius in range(1, radius_limit + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                candidates.append((center_x + dx * spacing_x, center_y + dy * spacing_y))
    return candidates


def is_position_free(
    candidate: PointTuple,
    occupied_positions: List[PointTuple],
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
