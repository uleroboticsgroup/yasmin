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
"""Clipboard and extraction data structures."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.layout import Position
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition

ContainerModel = StateMachine | Concurrence


@dataclass(slots=True)
class OutcomePlacementSnapshot:
    """Clipboard payload for one visual outcome alias."""

    outcome_name: str
    instance_id: str
    position: Position


@dataclass(slots=True)
class OutcomeRuleSnapshot:
    """Clipboard payload for one concurrence outcome-map rule."""

    outcome_name: str
    state_name: str
    state_outcome: str


@dataclass(slots=True)
class SelectionBundle:
    """Portable clipboard payload for selected editor items."""

    source_kind: str
    states: dict[str, State] = field(default_factory=dict)
    state_positions: dict[str, Position] = field(default_factory=dict)
    outcomes: dict[str, Outcome] = field(default_factory=dict)
    outcome_placements: list[OutcomePlacementSnapshot] = field(default_factory=list)
    text_blocks: list[TextBlock] = field(default_factory=list)
    transitions: list[Transition] = field(default_factory=list)
    transition_sources: list[str] = field(default_factory=list)
    outcome_rules: list[OutcomeRuleSnapshot] = field(default_factory=list)
    start_state: str | None = None

    @property
    def is_empty(self) -> bool:
        return not (
            self.states
            or self.outcomes
            or self.text_blocks
            or self.transitions
            or self.outcome_rules
        )

    def clone(self) -> "SelectionBundle":
        return copy.deepcopy(self)
