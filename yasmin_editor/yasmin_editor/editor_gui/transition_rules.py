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

from collections.abc import Collection, Sequence
from typing import List, Union
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.state_machine import StateMachine


class TransitionRuleError(ValueError):
    """Raised when a requested editor transition is not valid."""

    def __init__(self, message: str, *, title: str = "Error") -> None:
        super().__init__(message)
        self.title = title


def validate_drag_target(
    container_model,
    *,
    from_is_final_outcome: bool,
    to_is_final_outcome: bool,
) -> None:
    """Validate whether a dragged transition endpoint pair is allowed."""

    if isinstance(container_model, (Concurrence, OrthogonalState)):
        if from_is_final_outcome or not to_is_final_outcome:
            kind = (
                "Concurrence"
                if isinstance(container_model, Concurrence)
                else "OrthogonalState"
            )
            raise TransitionRuleError(
                f"States inside a {kind} can only connect to final outcomes of the current {kind}.",
                title="Not Allowed",
            )
        return

    if from_is_final_outcome:
        raise TransitionRuleError(
            "Final outcomes of the current container cannot start transitions here.",
            title="Not Allowed",
        )


def get_available_transition_outcomes(
    container_model,
    source_outcomes: Sequence[str],
    used_outcomes: Union[Collection[str], None] = None,
) -> List[str]:
    """Return the outcomes that may still create a transition from a source state."""

    if not source_outcomes:
        raise TransitionRuleError(
            "Cannot create transitions from states without outcomes!"
        )

    if not isinstance(container_model, (StateMachine, OrthogonalState)):
        return list(source_outcomes)

    if isinstance(container_model, OrthogonalState):
        return list(source_outcomes)

    used_outcomes = set(used_outcomes or ())
    available_outcomes = [
        outcome_name
        for outcome_name in source_outcomes
        if outcome_name not in used_outcomes
    ]
    if not available_outcomes:
        raise TransitionRuleError("All outcomes from this state are already used!")

    return available_outcomes
