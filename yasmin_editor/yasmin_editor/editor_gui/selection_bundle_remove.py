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

from typing import List, Set
from yasmin_editor.editor_gui.selection_models import ContainerModel
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.text_block import TextBlock


def remove_selection_from_model(
    container_model: ContainerModel,
    selected_state_names: Set[str],
    selected_outcome_instance_ids: Set[str],
    selected_text_blocks: List[TextBlock],
) -> None:
    """Remove the selected items from the source model."""

    for state_name in list(selected_state_names):
        container_model.remove_state(state_name)

    removed_outcome_names: Set[str] = set()
    for instance_id in list(selected_outcome_instance_ids):
        placement = container_model.layout.get_outcome_placement(instance_id)
        if placement is None:
            continue
        outcome_name = placement.outcome_name
        container_model.layout.remove_outcome_placement(instance_id)
        if not container_model.layout.get_outcome_placements(outcome_name):
            container_model.remove_outcome(outcome_name)
            removed_outcome_names.add(outcome_name)

    for text_block in selected_text_blocks:
        container_model.remove_text_block(text_block)

    if isinstance(container_model, (Concurrence, OrthogonalState)):
        for outcome_name in removed_outcome_names:
            container_model.outcome_map.pop(outcome_name, None)
