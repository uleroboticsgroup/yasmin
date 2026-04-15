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
"""Helpers for sizing the main editor window against screen geometry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True, slots=True)
class WindowRect:
    """Geometry rectangle used for initial window placement."""

    x: int
    y: int
    width: int
    height: int


DEFAULT_WINDOW_WIDTH = 1600
DEFAULT_WINDOW_HEIGHT = 950
MIN_WINDOW_WIDTH = 720
MIN_WINDOW_HEIGHT = 640
WINDOW_MARGIN = 24
WINDOW_WIDTH_RATIO = 0.94
WINDOW_HEIGHT_RATIO = 0.92


def _clamp_dimension(value: int, minimum: int, maximum: int) -> int:
    """Clamp one dimension to the supported range."""

    return max(minimum, min(value, maximum))


def rect_contains_point(rect: WindowRect, *, x: int, y: int) -> bool:
    """Return whether one global point lies inside the rectangle."""

    return rect.x <= x < rect.x + rect.width and rect.y <= y < rect.y + rect.height


def choose_preferred_screen_rect(
    screen_rects: Sequence[WindowRect],
    *,
    cursor_x: int | None,
    cursor_y: int | None,
    fallback_index: int = 0,
) -> WindowRect | None:
    """Return the screen rectangle that should host the initial editor window.

    The editor should open on the screen that currently contains the mouse
    cursor. When the cursor position is unavailable or outside all known
    screens, the function falls back to the window's current screen index.
    """

    if not screen_rects:
        return None

    if cursor_x is not None and cursor_y is not None:
        for rect in screen_rects:
            if rect_contains_point(rect, x=cursor_x, y=cursor_y):
                return rect

    clamped_index = max(0, min(fallback_index, len(screen_rects) - 1))
    return screen_rects[clamped_index]


def build_initial_window_rect(
    available_x: int,
    available_y: int,
    available_width: int,
    available_height: int,
    *,
    preferred_width: int = DEFAULT_WINDOW_WIDTH,
    preferred_height: int = DEFAULT_WINDOW_HEIGHT,
    minimum_width: int = MIN_WINDOW_WIDTH,
    minimum_height: int = MIN_WINDOW_HEIGHT,
    margin: int = WINDOW_MARGIN,
    width_ratio: float = WINDOW_WIDTH_RATIO,
    height_ratio: float = WINDOW_HEIGHT_RATIO,
) -> WindowRect:
    """Return a centered initial editor window rectangle inside the screen."""

    max_width = max(320, available_width - 2 * margin)
    max_height = max(240, available_height - 2 * margin)

    ratio_width = int(available_width * width_ratio)
    ratio_height = int(available_height * height_ratio)

    width = _clamp_dimension(
        min(preferred_width, ratio_width),
        min(minimum_width, max_width),
        max_width,
    )
    height = _clamp_dimension(
        min(preferred_height, ratio_height),
        min(minimum_height, max_height),
        max_height,
    )

    x = available_x + max(margin, (available_width - width) // 2)
    y = available_y + max(margin, (available_height - height) // 2)

    max_x = available_x + available_width - width
    max_y = available_y + available_height - height
    x = min(x, max_x)
    y = min(y, max_y)

    return WindowRect(x=x, y=y, width=width, height=height)
