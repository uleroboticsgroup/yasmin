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
"""State machine model."""

from __future__ import annotations

from dataclasses import dataclass, field

from .layout import Layout
from .state import State
from .transition import Transition


@dataclass(slots=True, repr=False)
class StateMachine(State):
    """Represents a YASMIN state machine container."""

    start_state: str | None = None
    states: dict[str, State] = field(default_factory=dict)
    transitions: dict[str, list[Transition]] = field(default_factory=dict)
    remappings: dict[str, dict[str, str]] = field(default_factory=dict)
    layout: Layout = field(default_factory=Layout)

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""

        return True

    def add_state(self, state: State) -> None:
        """Add a child state to the state machine."""

        self.states[state.name] = state
        self.transitions.setdefault(state.name, [])
        self.remappings.setdefault(state.name, {})

    def add_transition(self, state_name: str, transition: Transition) -> None:
        """Add a transition for a child state."""

        self.transitions.setdefault(state_name, []).append(transition)

    def set_remapping(self, state_name: str, source: str, target: str) -> None:
        """Set a remapping for a child state inside this state machine."""

        self.remappings.setdefault(state_name, {})[source] = target

    def get_state(self, name: str) -> State | None:
        """Return a child state by name."""

        return self.states.get(name)

    def to_string(self, indent: int = 0) -> str:
        """Return a human-readable representation of the state machine."""

        prefix = " " * indent
        lines: list[str] = []

        header = f"{prefix}StateMachine(name={self.name!r}"
        if self.start_state:
            header += f", start_state={self.start_state!r}"
        if self.outcomes:
            header += ", outcomes=[{}]".format(
                ", ".join(outcome.name for outcome in self.outcomes)
            )
        if self.keys:
            header += ", keys=[{}]".format(", ".join(key.name for key in self.keys))
        header += ")"
        lines.append(header)

        if self.description:
            lines.append(f"{prefix}  description: {self.description}")

        if self.states:
            lines.append(f"{prefix}  states:")
            for state_name, state in self.states.items():
                if state.is_container:
                    lines.append(f"{prefix}    - {state.name}")
                    lines.append(state.to_string(indent + 8))
                else:
                    lines.append(f"{prefix}    - {state._format_header()}")

                state_transitions = self.transitions.get(state_name, [])
                for transition in state_transitions:
                    lines.append(
                        f"{prefix}      {transition.source_outcome} -> {transition.target}"
                    )

                state_remappings = self.remappings.get(state_name, {})
                if state_remappings:
                    lines.append(
                        f"{prefix}      remap: "
                        + ", ".join(
                            f"{source}->{target}"
                            for source, target in state_remappings.items()
                        )
                    )

        if self.outcomes:
            lines.append(f"{prefix}  final outcomes:")
            for outcome in self.outcomes:
                if outcome.description:
                    lines.append(f"{prefix}    - {outcome.name}: {outcome.description}")
                else:
                    lines.append(f"{prefix}    - {outcome.name}")

        container_transitions = self.transitions.get(self.name, [])
        if container_transitions:
            lines.append(f"{prefix}  container transitions:")
            for transition in container_transitions:
                lines.append(
                    f"{prefix}    {transition.source_outcome} -> {transition.target}"
                )

        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""

        return self.to_string()

    __repr__ = __str__
