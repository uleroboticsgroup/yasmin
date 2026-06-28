# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from __future__ import annotations

from dataclasses import dataclass

from .outcome import Outcome
from .state import State


@dataclass(slots=True, repr=False)
class JoinState(State):
    """Represents a YASMIN join-state leaf that synchronizes region threads."""

    sync_id: str = ""
    join_outcome: str = "joined"

    def __post_init__(self) -> None:
        if not self.outcomes:
            self.outcomes = [Outcome(name=self.join_outcome)]

    def __str__(self) -> str:
        parts = [f"JoinState({self.name})"]
        if self.sync_id:
            parts.append(f"sync_id={self.sync_id}")
        parts.append(f"outcome={self.join_outcome}")
        return " ".join(parts)

    __repr__ = __str__
