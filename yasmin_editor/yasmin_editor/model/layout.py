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
from uuid import uuid4


@dataclass(slots=True)
class Position:
    """Represents a 2D position in the editor canvas."""

    x: float = 0.0
    y: float = 0.0


@dataclass(slots=True)
class OutcomePlacement:
    """Stores one visual placement for a logical final outcome."""

    instance_id: str
    outcome_name: str
    position: Position


@dataclass(slots=True)
class Layout:
    """Stores positions for child states and final outcomes inside a container."""

    state_positions: dict[str, Position] = field(default_factory=dict)
    outcome_positions: dict[str, Position] = field(default_factory=dict)
    outcome_placements: dict[str, OutcomePlacement] = field(default_factory=dict)

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

    def _create_instance_id(self, outcome_name: str) -> str:
        safe_name = outcome_name.strip() or "outcome"
        return f"{safe_name}-{uuid4().hex[:8]}"

    def get_outcome_placements(
        self,
        outcome_name: str | None = None,
    ) -> list[OutcomePlacement]:
        """Return all stored outcome placements, optionally filtered by outcome name."""

        placements = list(self.outcome_placements.values())
        if outcome_name is not None:
            placements = [
                placement
                for placement in placements
                if placement.outcome_name == outcome_name
            ]
        return placements

    def get_outcome_placement(self, instance_id: str) -> OutcomePlacement | None:
        """Return one outcome placement by its instance identifier."""

        return self.outcome_placements.get(instance_id)

    def ensure_outcome_placement(
        self,
        outcome_name: str,
        x: float,
        y: float,
        instance_id: str | None = None,
    ) -> str:
        """Create or update one visual placement for a logical outcome."""

        instance_id = instance_id or self._create_instance_id(outcome_name)
        self.outcome_placements[instance_id] = OutcomePlacement(
            instance_id=instance_id,
            outcome_name=outcome_name,
            position=Position(x, y),
        )
        self._sync_primary_outcome_position(outcome_name)
        return instance_id

    def create_outcome_alias(
        self,
        name: str,
        x: float,
        y: float,
    ) -> str:
        """Create a new visual alias for an existing logical outcome."""

        return self.ensure_outcome_placement(name, x, y, instance_id=None)

    def set_outcome_position(
        self,
        name: str,
        x: float,
        y: float,
        instance_id: str | None = None,
    ) -> str:
        """Update one stored final-outcome placement.

        Without an ``instance_id`` this updates the primary visible alias. Use
        :meth:`create_outcome_alias` when a new visual alias should be created.
        """

        if instance_id is None:
            placements = self.get_outcome_placements(name)
            if placements:
                instance_id = placements[0].instance_id
        return self.ensure_outcome_placement(name, x, y, instance_id=instance_id)

    def get_outcome_position(self, name: str) -> Position | None:
        """Return the primary position of a logical final outcome if present."""

        placements = self.get_outcome_placements(name)
        if placements:
            return placements[0].position
        return self.outcome_positions.get(name)

    def set_primary_outcome_position(self, name: str, x: float, y: float) -> None:
        """Store the primary legacy outcome position without creating an alias.

        Legacy XML can describe one final outcome position without an explicit
        ``instance_id``. The editor still supports that representation for
        stable roundtrips, so callers that intentionally want a non-alias
        primary placement should use this method instead of
        :meth:`set_outcome_position`.
        """

        self.outcome_positions[name] = Position(x, y)

    def materialize_primary_outcome_position(self, name: str) -> str | None:
        """Convert one legacy primary position into an explicit alias instance.

        This is useful when a layout initially loaded one legacy outcome
        placement without an ``instance_id`` and later needs more than one
        visual alias for the same logical outcome.
        """

        if self.get_outcome_placements(name):
            return None
        primary_position = self.outcome_positions.get(name)
        if primary_position is None:
            return None
        instance_id = self.create_outcome_alias(
            name,
            primary_position.x,
            primary_position.y,
        )
        return instance_id

    def remove_outcome_placement(self, instance_id: str) -> None:
        """Remove one visual placement of a final outcome."""

        placement = self.outcome_placements.pop(instance_id, None)
        if placement is not None:
            self._sync_primary_outcome_position(placement.outcome_name)

    def remove_outcome_position(self, name: str) -> None:
        """Remove all stored placements of a logical final outcome."""

        self.outcome_positions.pop(name, None)
        for instance_id in [
            placement.instance_id for placement in self.get_outcome_placements(name)
        ]:
            self.outcome_placements.pop(instance_id, None)

    def rename_outcome_position(self, old_name: str, new_name: str) -> None:
        """Rename the stored placements of a logical final outcome."""

        position = self.outcome_positions.pop(old_name, None)
        if position is not None:
            self.outcome_positions[new_name] = position

        for placement in self.outcome_placements.values():
            if placement.outcome_name == old_name:
                placement.outcome_name = new_name

        self._sync_primary_outcome_position(new_name)

    def _sync_primary_outcome_position(self, outcome_name: str) -> None:
        placements = self.get_outcome_placements(outcome_name)
        if not placements:
            self.outcome_positions.pop(outcome_name, None)
            return
        primary_position = placements[0].position
        self.outcome_positions[outcome_name] = Position(
            primary_position.x,
            primary_position.y,
        )
