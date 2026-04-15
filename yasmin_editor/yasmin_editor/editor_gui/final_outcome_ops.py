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
"""Model helpers for logical final outcomes and their visual aliases."""

from __future__ import annotations

from dataclasses import dataclass

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome


@dataclass(frozen=True, slots=True)
class FinalOutcomeAliasResult:
    """Result of ensuring a visible final-outcome alias in a container."""

    outcome: Outcome
    instance_id: str
    created_outcome: bool


def ensure_final_outcome_alias(
    container_model,
    outcome_name: str,
    x: float,
    y: float,
) -> FinalOutcomeAliasResult:
    """Ensure one logical final outcome exists and add a visible alias for it."""

    outcome = next(
        (item for item in container_model.outcomes if item.name == outcome_name),
        None,
    )
    created_outcome = outcome is None
    if outcome is None:
        outcome = Outcome(name=outcome_name)
        container_model.add_outcome(outcome)

    instance_id = container_model.layout.create_outcome_alias(outcome_name, x, y)

    if (
        isinstance(container_model, Concurrence)
        and len(container_model.outcomes) == 1
        and not container_model.default_outcome
    ):
        container_model.default_outcome = outcome_name

    return FinalOutcomeAliasResult(
        outcome=outcome,
        instance_id=instance_id,
        created_outcome=created_outcome,
    )
