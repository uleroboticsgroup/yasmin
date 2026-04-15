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
"""Pure helper functions for container metadata panels and rename rules."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine


@dataclass(frozen=True, slots=True)
class ContainerMetadataView:
    """UI-ready metadata for the currently visible container."""

    name_label_html: str
    selector_label_html: str
    selector_items: list[str]
    current_selector_value: str | None


ContainerModel = StateMachine | Concurrence


def build_container_metadata_view(model: ContainerModel) -> ContainerMetadataView:
    """Return the labels and selector entries for one container metadata panel."""

    if isinstance(model, StateMachine):
        return ContainerMetadataView(
            name_label_html="<b>State Machine Name:</b>",
            selector_label_html="<b>Start State:</b>",
            selector_items=sorted(model.states.keys()),
            current_selector_value=model.start_state,
        )

    return ContainerMetadataView(
        name_label_html="<b>Concurrence Name:</b>",
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
