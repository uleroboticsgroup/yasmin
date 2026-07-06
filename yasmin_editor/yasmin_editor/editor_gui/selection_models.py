# Copyright (C) 2026 Maik Knof
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import copy
from typing import List, Dict, Union
from yasmin_editor.dataclass_compat import dataclass, field
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.layout import Position
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition

ContainerModel = Union[StateMachine, Concurrence, OrthogonalState]


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
    states: Dict[str, State] = field(default_factory=dict)
    state_positions: Dict[str, Position] = field(default_factory=dict)
    outcomes: Dict[str, Outcome] = field(default_factory=dict)
    outcome_placements: List[OutcomePlacementSnapshot] = field(default_factory=list)
    text_blocks: List[TextBlock] = field(default_factory=list)
    transitions: List[Transition] = field(default_factory=list)
    transition_sources: List[str] = field(default_factory=list)
    outcome_rules: List[OutcomeRuleSnapshot] = field(default_factory=list)
    start_state: Union[str, None] = None

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
