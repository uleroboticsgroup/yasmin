"""Outcome model."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class Outcome:
    """Represents an outcome of a state or container."""

    name: str
    description: str = ""
