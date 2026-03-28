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

    def set_outcome_position(self, name: str, x: float, y: float) -> None:
        """Set the position of a final outcome."""

        self.outcome_positions[name] = Position(x, y)

    def get_outcome_position(self, name: str) -> Position | None:
        """Return the position of a final outcome if present."""

        return self.outcome_positions.get(name)
