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

from yasmin_editor.dataclass_compat import dataclass
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
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
        isinstance(container_model, (Concurrence, OrthogonalState))
        and len(container_model.outcomes) == 1
        and not container_model.default_outcome
    ):
        container_model.default_outcome = outcome_name

    return FinalOutcomeAliasResult(
        outcome=outcome,
        instance_id=instance_id,
        created_outcome=created_outcome,
    )
