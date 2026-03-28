"""Transition model."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class Transition:
    """Represents a transition from one outcome to a target state or final outcome."""

    source_outcome: str
    target: str
