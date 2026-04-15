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
from .text_block import TextBlock

OutcomeRuleValues = str | list[str]


def iter_outcome_rule_values(values: OutcomeRuleValues) -> list[str]:
    """Return one normalized list of unique child outcomes."""

    if isinstance(values, str):
        return [values] if values else []

    normalized: list[str] = []
    for outcome_name in values:
        if outcome_name and outcome_name not in normalized:
            normalized.append(outcome_name)
    return normalized


@dataclass(slots=True, repr=False)
class Concurrence(State):
    """Represents a YASMIN concurrence container."""

    default_outcome: str | None = None
    states: dict[str, State] = field(default_factory=dict)
    outcome_map: dict[str, dict[str, list[str]]] = field(default_factory=dict)
    layout: Layout = field(default_factory=Layout)
    text_blocks: list[TextBlock] = field(default_factory=list)

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
                f"Name '{name}' is already used by a child state in this concurrence"
            )
        if any(
            outcome.name == name and outcome.name != exclude_outcome
            for outcome in self.outcomes
        ):
            raise ValueError(
                f"Name '{name}' is already used by a final outcome in this concurrence"
            )

    def add_state(self, state: State) -> None:
        """Add a child state to the concurrence."""
        self._assert_child_name_available(state.name)
        self.states[state.name] = state

    def add_outcome(self, outcome) -> None:
        """Add a final outcome to the concurrence."""
        self._assert_child_name_available(outcome.name)
        State.add_outcome(self, outcome)

    def add_text_block(self, text_block: TextBlock) -> None:
        """Add a free-form text block to the container."""
        self.text_blocks.append(text_block)

    def remove_text_block(self, text_block: TextBlock) -> None:
        """Remove one free-form text block from the container."""
        self.text_blocks = [item for item in self.text_blocks if item is not text_block]

    def set_outcome_rule(
        self,
        outcome: str,
        state_name: str,
        state_outcome: str,
    ) -> None:
        """Set one outcome rule entry for the concurrence."""
        mapping = self.outcome_map.setdefault(outcome, {})
        state_outcomes = iter_outcome_rule_values(mapping.get(state_name, []))
        if state_outcome not in state_outcomes:
            state_outcomes.append(state_outcome)
        mapping[state_name] = state_outcomes

    def remove_outcome_rule(
        self,
        outcome: str,
        state_name: str,
        state_outcome: str | None = None,
    ) -> None:
        """Remove one outcome rule entry from the concurrence."""
        mapping = self.outcome_map.get(outcome, {})
        if state_outcome is None:
            mapping.pop(state_name, None)
        else:
            state_outcomes = iter_outcome_rule_values(mapping.get(state_name, []))
            state_outcomes = [item for item in state_outcomes if item != state_outcome]
            if state_outcomes:
                mapping[state_name] = state_outcomes
            else:
                mapping.pop(state_name, None)
        if not mapping:
            self.outcome_map.pop(outcome, None)

    def remove_state(self, name: str) -> None:
        """Remove a child state and all related outcome rules."""
        self.states.pop(name, None)
        for mapping in self.outcome_map.values():
            mapping.pop(name, None)
        empty_outcomes = [
            outcome for outcome, mapping in self.outcome_map.items() if not mapping
        ]
        for outcome in empty_outcomes:
            self.outcome_map.pop(outcome, None)
        self.layout.remove_state_position(name)

    def rename_state(self, old_name: str, new_name: str) -> None:
        """Rename a child state and update all related outcome rules."""
        if old_name == new_name:
            return
        self._assert_child_name_available(new_name, exclude_state=old_name)
        state = self.states.pop(old_name)
        state.name = new_name
        self.states[new_name] = state
        for mapping in self.outcome_map.values():
            if old_name in mapping:
                mapping[new_name] = iter_outcome_rule_values(mapping.pop(old_name))
        self.layout.rename_state_position(old_name, new_name)

    def rename_outcome(self, old_name: str, new_name: str) -> None:
        """Rename a final outcome and update all related references."""
        if old_name == new_name:
            return
        if self.get_outcome(old_name) is None:
            return
        self._assert_child_name_available(new_name, exclude_outcome=old_name)
        State.rename_outcome(self, old_name, new_name)
        mapping = self.outcome_map.pop(old_name, None)
        if mapping is not None:
            self.outcome_map[new_name] = {
                state_name: iter_outcome_rule_values(state_outcomes)
                for state_name, state_outcomes in mapping.items()
            }
        if self.default_outcome == old_name:
            self.default_outcome = new_name
        self.layout.rename_outcome_position(old_name, new_name)

    def rename_child_state_outcome(
        self,
        state_name: str,
        old_outcome: str,
        new_outcome: str,
    ) -> None:
        """Rename one child-state outcome used by the outcome map."""
        if old_outcome == new_outcome:
            return
        for mapping in self.outcome_map.values():
            state_outcomes = mapping.get(state_name)
            if state_outcomes is None:
                continue
            renamed_outcomes: list[str] = []
            for outcome_name in iter_outcome_rule_values(state_outcomes):
                candidate = new_outcome if outcome_name == old_outcome else outcome_name
                if candidate not in renamed_outcomes:
                    renamed_outcomes.append(candidate)
            if renamed_outcomes:
                mapping[state_name] = renamed_outcomes
            else:
                mapping.pop(state_name, None)

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
        if self.text_blocks:
            lines.append(f"{prefix}  text blocks:")
            for text_block in self.text_blocks:
                preview = text_block.content.replace("\n", "\\n")
                lines.append(
                    f"{prefix}    - ({text_block.x:.2f}, {text_block.y:.2f}): {preview}"
                )
        if self.outcome_map:
            lines.append(f"{prefix}  outcome map:")
            for outcome_name, mapping in self.outcome_map.items():
                rules = ", ".join(
                    f"{state_name}={state_outcome}"
                    for state_name, state_outcomes in mapping.items()
                    for state_outcome in iter_outcome_rule_values(state_outcomes)
                )
                lines.append(f"{prefix}    - {outcome_name}: {rules}")
        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""
        return self.to_string()

    __repr__ = __str__
