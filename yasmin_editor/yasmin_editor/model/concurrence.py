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
"""Concurrence model."""

from __future__ import annotations

from dataclasses import dataclass, field

from .layout import Layout
from .state import State


@dataclass(slots=True, repr=False)
class Concurrence(State):
    """Represents a YASMIN concurrence container."""

    default_outcome: str | None = None
    states: dict[str, State] = field(default_factory=dict)
    outcome_map: dict[str, dict[str, str]] = field(default_factory=dict)
    layout: Layout = field(default_factory=Layout)

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""

        return True

    def add_state(self, state: State) -> None:
        """Add a child state to the concurrence."""

        self.states[state.name] = state

    def set_outcome_rule(
        self,
        outcome: str,
        state_name: str,
        state_outcome: str,
    ) -> None:
        """Set one outcome rule entry for the concurrence."""

        self.outcome_map.setdefault(outcome, {})[state_name] = state_outcome

    def remove_outcome_rule(self, outcome: str, state_name: str) -> None:
        """Remove one outcome rule entry from the concurrence."""

        mapping = self.outcome_map.get(outcome, {})
        mapping.pop(state_name, None)
        if not mapping:
            self.outcome_map.pop(outcome, None)

    def remove_state(self, name: str) -> None:
        """Remove a child state and all related outcome rules."""

        self.states.pop(name, None)
        for mapping in self.outcome_map.values():
            mapping.pop(name, None)
        empty_outcomes = [outcome for outcome, mapping in self.outcome_map.items() if not mapping]
        for outcome in empty_outcomes:
            self.outcome_map.pop(outcome, None)
        self.layout.remove_state_position(name)

    def rename_state(self, old_name: str, new_name: str) -> None:
        """Rename a child state and update all related outcome rules."""

        if old_name == new_name:
            return

        state = self.states.pop(old_name)
        state.name = new_name
        self.states[new_name] = state

        for mapping in self.outcome_map.values():
            if old_name in mapping:
                mapping[new_name] = mapping.pop(old_name)

        self.layout.rename_state_position(old_name, new_name)

    def remove_outcome(self, name: str) -> None:
        """Remove a final outcome and its outcome-map rule."""

        self.outcomes = [outcome for outcome in self.outcomes if outcome.name != name]
        self.outcome_map.pop(name, None)
        if self.default_outcome == name:
            self.default_outcome = None
        self.layout.remove_outcome_position(name)

    def get_state(self, name: str) -> State | None:
        """Return a child state by name."""

        return self.states.get(name)

    def to_string(self, indent: int = 0) -> str:
        """Return a human-readable representation of the concurrence."""

        prefix = " " * indent
        lines: list[str] = []

        header = f"{prefix}Concurrence(name={self.name!r}"
        if self.default_outcome:
            header += f", default_outcome={self.default_outcome!r}"
        if self.outcomes:
            header += ", outcomes=[{}]".format(
                ", ".join(outcome.name for outcome in self.outcomes)
            )
        header += ")"
        lines.append(header)

        if self.description:
            lines.append(f"{prefix}  description: {self.description}")

        if self.states:
            lines.append(f"{prefix}  states:")
            for state in self.states.values():
                if state.is_container:
                    lines.append(f"{prefix}    - {state.name}")
                    lines.append(state.to_string(indent + 8))
                else:
                    lines.append(f"{prefix}    - {state._format_header()}")

        if self.outcome_map:
            lines.append(f"{prefix}  outcome map:")
            for outcome_name, mapping in self.outcome_map.items():
                rules = ", ".join(
                    f"{state_name}={state_outcome}"
                    for state_name, state_outcome in mapping.items()
                )
                lines.append(f"{prefix}    - {outcome_name}: {rules}")

        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""

        return self.to_string()

    __repr__ = __str__
