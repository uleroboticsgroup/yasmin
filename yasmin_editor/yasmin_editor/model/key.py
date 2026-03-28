"""Blackboard key model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class Key:
    """Represents a blackboard key definition."""

    name: str
    key_type: str = "in"
    description: str = ""
    default_type: str = ""
    default_value: Any = None

    @property
    def has_default(self) -> bool:
        """Return whether a default value is defined for this key."""

        return self.default_type != ""
