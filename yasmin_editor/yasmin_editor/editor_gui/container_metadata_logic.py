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

from typing import Iterable, Sequence, List, Union

from yasmin_editor.dataclass_compat import dataclass
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.state_machine import StateMachine


@dataclass(frozen=True, slots=True)
class ContainerMetadataView:
    """UI-ready metadata for the currently visible container."""

    name_label_html: str
    selector_label_html: str
    selector_items: List[str]
    current_selector_value: Union[str, None]


ContainerModel = Union[StateMachine, Concurrence, OrthogonalState]


def build_container_metadata_view(model: ContainerModel) -> ContainerMetadataView:
    """Return the labels and selector entries for one container metadata panel."""

    if isinstance(model, StateMachine):
        return ContainerMetadataView(
            name_label_html="<b>State Machine Name:</b>",
            selector_label_html="<b>Start State:</b>",
            selector_items=sorted(model.states),
            current_selector_value=model.start_state,
        )

    kind = "Orthogonal State" if isinstance(model, OrthogonalState) else "Concurrence"
    return ContainerMetadataView(
        name_label_html=f"<b>{kind} Name:</b>",
        selector_label_html="<b>Default Outcome:</b>",
        selector_items=[outcome.name for outcome in model.outcomes],
        current_selector_value=model.default_outcome,
    )


def normalize_container_name(text: str) -> str:
    """Return the trimmed container name entered by the user."""

    return text.strip()


def has_container_name_conflict(
    proposed_name: str,
    *,
    current_name: str,
    sibling_state_names: Iterable[str],
    sibling_outcome_names: Sequence[str],
) -> bool:
    """Return whether a rename would collide with sibling states or outcomes."""

    if proposed_name == current_name:
        return False
    return proposed_name in set(sibling_state_names) or proposed_name in set(
        sibling_outcome_names
    )
