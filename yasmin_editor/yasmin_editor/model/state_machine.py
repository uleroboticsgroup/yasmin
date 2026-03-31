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
    layout: Layout = field(default_factory=Layout)

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""
        return True

    def _assert_child_name_available(
        self,
        name: str,
        *,
        exclude_state: str | None = None,
        exclude_outcome: str | None = None,
    ) -> None:
        """Validate that a child state or final outcome name is available."""
        if name in self.states and name != exclude_state:
            raise ValueError(
                f"Name '{name}' is already used by a child state in this state machine"
            )
        if any(
            outcome.name == name and outcome.name != exclude_outcome
            for outcome in self.outcomes
        ):
            raise ValueError(
                f"Name '{name}' is already used by a final outcome in this state machine"
            )

    def add_state(self, state: State) -> None:
        """Add a child state to the state machine."""
        self._assert_child_name_available(state.name)
        self.states[state.name] = state
        self.transitions.setdefault(state.name, [])

    def add_outcome(self, outcome) -> None:
        """Add a final outcome to the state machine."""
        self._assert_child_name_available(outcome.name)
        State.add_outcome(self, outcome)

    def add_transition(self, state_name: str, transition: Transition) -> None:
        """Add a transition for a child state."""
        transition_list = self.transitions.setdefault(state_name, [])
        for item in transition_list:
            if (
                item.source_outcome == transition.source_outcome
                and item.target == transition.target
            ):
                return
        transition_list.append(transition)

    def remove_transition(
        self,
        state_name: str,
        source_outcome: str,
        target: str,
    ) -> None:
        """Remove a transition for a child state."""
        transitions = self.transitions.get(state_name, [])
        self.transitions[state_name] = [
            item
            for item in transitions
            if not (item.source_outcome == source_outcome and item.target == target)
        ]
        if not self.transitions.get(state_name):
            self.transitions.pop(state_name, None)

    def remove_state(self, name: str) -> None:
        """Remove a child state and all related transitions."""
        self.states.pop(name, None)
        self.transitions.pop(name, None)
        for transitions in self.transitions.values():
            transitions[:] = [item for item in transitions if item.target != name]
        if self.start_state == name:
            self.start_state = None
        self.layout.remove_state_position(name)

    def rename_transition_owner(self, old_owner: str, new_owner: str) -> None:
        """Rename one transition owner entry inside this container."""
        if old_owner == new_owner:
            return
        old_transitions = self.transitions.pop(old_owner, [])
        if not old_transitions:
            return
        merged = self.transitions.setdefault(new_owner, [])
        for transition in old_transitions:
            if any(
                item.source_outcome == transition.source_outcome
                and item.target == transition.target
                for item in merged
            ):
                continue
            merged.append(transition)

    def rename_state(self, old_name: str, new_name: str) -> None:
        """Rename a child state and update all related references."""
        if old_name == new_name:
            return
        self._assert_child_name_available(new_name, exclude_state=old_name)
        state = self.states.pop(old_name)
        state.name = new_name
        self.states[new_name] = state
        self.rename_transition_owner(old_name, new_name)
        for transitions in self.transitions.values():
            for transition in transitions:
                if transition.target == old_name:
                    transition.target = new_name
        if isinstance(state, StateMachine):
            state.rename_transition_owner(old_name, new_name)
        if self.start_state == old_name:
            self.start_state = new_name
        self.layout.rename_state_position(old_name, new_name)

    def rename_outcome(self, old_name: str, new_name: str) -> None:
        """Rename a final outcome and update all related references."""
        if old_name == new_name:
            return
        self._assert_child_name_available(new_name, exclude_outcome=old_name)
        State.rename_outcome(self, old_name, new_name)
        self.rename_transition_owner(old_name, new_name)
        for transitions in self.transitions.values():
            for transition in transitions:
                if transition.target == old_name:
                    transition.target = new_name
        self.layout.rename_outcome_position(old_name, new_name)

    def rename_child_state_outcome(
        self,
        state_name: str,
        old_outcome: str,
        new_outcome: str,
    ) -> None:
        """Rename one outcome referenced by transitions of a child state."""
        if old_outcome == new_outcome:
            return
        transitions = self.transitions.get(state_name, [])
        if not transitions:
            return
        deduplicated: list[Transition] = []
        seen: set[tuple[str, str]] = set()
        for transition in transitions:
            source_outcome = (
                new_outcome
                if transition.source_outcome == old_outcome
                else transition.source_outcome
            )
            key = (source_outcome, transition.target)
            if key in seen:
                continue
            seen.add(key)
            transition.source_outcome = source_outcome
            deduplicated.append(transition)
        if deduplicated:
            self.transitions[state_name] = deduplicated
        else:
            self.transitions.pop(state_name, None)

    def remove_outcome(self, name: str) -> None:
        """Remove a final outcome and all related transitions."""
        self.outcomes = [outcome for outcome in self.outcomes if outcome.name != name]
        self.transitions.pop(name, None)
        for transitions in self.transitions.values():
            transitions[:] = [item for item in transitions if item.target != name]
        self.layout.remove_outcome_position(name)

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
                if state.remappings:
                    lines.append(
                        f"{prefix}      remap: "
                        + ", ".join(
                            f"{source}->{target}"
                            for source, target in state.remappings.items()
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
