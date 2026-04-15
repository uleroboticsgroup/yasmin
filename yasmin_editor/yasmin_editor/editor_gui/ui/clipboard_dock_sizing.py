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
"""Helpers for persisting the user-controlled shelf width."""

from __future__ import annotations

DEFAULT_CLIPBOARD_DOCK_WIDTH = 280
MIN_CLIPBOARD_DOCK_WIDTH = 180


def normalize_persisted_clipboard_dock_width(
    persisted_width: object,
    *,
    minimum_width: int = MIN_CLIPBOARD_DOCK_WIDTH,
    default_width: int = DEFAULT_CLIPBOARD_DOCK_WIDTH,
) -> int:
    """Return one persisted shelf width value as a safe integer.

    Settings backends may return strings, floats, or invalid values. The editor
    should always recover to a readable default instead of propagating malformed
    persisted state into the dock geometry.
    """

    try:
        width = int(persisted_width)
    except (TypeError, ValueError):
        width = int(default_width)
    return clamp_clipboard_dock_width(width, minimum_width=minimum_width)


def clamp_clipboard_dock_width(
    target_width: int,
    *,
    minimum_width: int = MIN_CLIPBOARD_DOCK_WIDTH,
) -> int:
    """Clamp one requested shelf width to the supported lower bound."""

    return max(minimum_width, int(target_width))
