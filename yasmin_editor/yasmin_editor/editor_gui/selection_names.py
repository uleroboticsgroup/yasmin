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

import re
from typing import Set

_TRAILING_NUMBER_PATTERN = re.compile(r"^(.*?)(\d+)$")


def increment_name(base_name: str, existing_names: Set[str]) -> str:
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
