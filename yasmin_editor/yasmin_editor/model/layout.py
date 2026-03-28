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
"""Container layout model."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class Position:
    """Represents a 2D position in the editor canvas."""

    x: float = 0.0
    y: float = 0.0


@dataclass(slots=True)
class Layout:
    """Stores positions for child states and final outcomes inside a container."""

    state_positions: dict[str, Position] = field(default_factory=dict)
    outcome_positions: dict[str, Position] = field(default_factory=dict)

    def set_state_position(self, name: str, x: float, y: float) -> None:
        """Set the position of a child state."""

        self.state_positions[name] = Position(x, y)

    def get_state_position(self, name: str) -> Position | None:
        """Return the position of a child state if present."""

        return self.state_positions.get(name)

    def remove_state_position(self, name: str) -> None:
        """Remove the stored position of a child state."""

        self.state_positions.pop(name, None)

    def rename_state_position(self, old_name: str, new_name: str) -> None:
        """Rename the stored position of a child state."""

        position = self.state_positions.pop(old_name, None)
        if position is not None:
            self.state_positions[new_name] = position

    def set_outcome_position(self, name: str, x: float, y: float) -> None:
        """Set the position of a final outcome."""

        self.outcome_positions[name] = Position(x, y)

    def get_outcome_position(self, name: str) -> Position | None:
        """Return the position of a final outcome if present."""

        return self.outcome_positions.get(name)

    def remove_outcome_position(self, name: str) -> None:
        """Remove the stored position of a final outcome."""

        self.outcome_positions.pop(name, None)

    def rename_outcome_position(self, old_name: str, new_name: str) -> None:
        """Rename the stored position of a final outcome."""

        position = self.outcome_positions.pop(old_name, None)
        if position is not None:
            self.outcome_positions[new_name] = position
