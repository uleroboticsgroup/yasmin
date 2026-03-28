"""Base state model."""

from __future__ import annotations

from dataclasses import dataclass, field

from .key import Key
from .outcome import Outcome


@dataclass(slots=True, repr=False)
class State:
    """Base model for all YASMIN states."""

    name: str
    description: str = ""
    keys: list[Key] = field(default_factory=list)
    outcomes: list[Outcome] = field(default_factory=list)
    state_type: str | None = None
    module: str | None = None
    class_name: str | None = None
    package_name: str | None = None
    file_name: str | None = None

    def add_key(self, key: Key) -> None:
        """Add a blackboard key to the state."""

        self.keys.append(key)

    def add_outcome(self, outcome: Outcome) -> None:
        """Add an outcome to the state."""

        self.outcomes.append(outcome)

    def get_outcome(self, name: str) -> Outcome | None:
        """Return an outcome by name."""

        for outcome in self.outcomes:
            if outcome.name == name:
                return outcome
        return None

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""

        return False

    @property
    def is_leaf(self) -> bool:
        """Return whether this state is a leaf state."""

        return not self.is_container

    def _format_header(self) -> str:
        """Return a compact one-line header for this state."""

        parts: list[str] = [self.name]

        meta: list[str] = []
        if self.state_type:
            meta.append(f"type={self.state_type}")
        if self.class_name:
            meta.append(f"class={self.class_name}")
        if self.module:
            meta.append(f"module={self.module}")
        if self.package_name:
            meta.append(f"package={self.package_name}")
        if self.file_name:
            meta.append(f"file={self.file_name}")

        if meta:
            parts.append(f"({', '.join(meta)})")

        return " ".join(parts)

    def to_string(self, indent: int = 0) -> str:
        """Return a human-readable representation of the state."""

        prefix = " " * indent
        lines = [f"{prefix}State({self._format_header()})"]

        if self.outcomes:
            lines.append(
                f"{prefix}  outcomes: {', '.join(outcome.name for outcome in self.outcomes)}"
            )

        if self.keys:
            lines.append(f"{prefix}  keys: {', '.join(key.name for key in self.keys)}")

        if self.description:
            lines.append(f"{prefix}  description: {self.description}")

        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""

        return self.to_string()

    __repr__ = __str__
