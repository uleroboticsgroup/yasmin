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
"""Pure transition-validation helpers for the editor canvas."""

from __future__ import annotations

from collections.abc import Collection, Sequence

from yasmin_editor.model.concurrence import Concurrence
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

    if isinstance(container_model, Concurrence):
        if from_is_final_outcome or not to_is_final_outcome:
            raise TransitionRuleError(
                "States inside a Concurrence can only connect to final outcomes of the current Concurrence.",
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
    used_outcomes: Collection[str] | None = None,
) -> list[str]:
    """Return the outcomes that may still create a transition from a source state."""

    if not source_outcomes:
        raise TransitionRuleError(
            "Cannot create transitions from states without outcomes!"
        )

    if not isinstance(container_model, StateMachine):
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
