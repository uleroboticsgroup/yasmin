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
"""Naming helpers for duplicated editor objects."""

from __future__ import annotations

import re

_TRAILING_NUMBER_PATTERN = re.compile(r"^(.*?)(\d+)$")


def increment_name(base_name: str, existing_names: set[str]) -> str:
    """Return a name that follows the requested 2, 3, 4... suffix rule."""

    candidate = base_name.strip() or "state"
    if candidate not in existing_names:
        return candidate

    match = _TRAILING_NUMBER_PATTERN.match(candidate)
    if match:
        prefix = match.group(1)
        next_number = int(match.group(2)) + 1
    else:
        prefix = candidate
        next_number = 2

    while True:
        updated = f"{prefix}{next_number}"
        if updated not in existing_names:
            return updated
        next_number += 1
