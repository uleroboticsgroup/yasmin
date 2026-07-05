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
