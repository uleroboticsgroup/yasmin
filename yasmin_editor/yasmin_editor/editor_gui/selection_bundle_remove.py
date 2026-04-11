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
"""Removal helpers for selection-based editor operations."""

from __future__ import annotations

from yasmin_editor.editor_gui.selection_models import ContainerModel
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.text_block import TextBlock


def remove_selection_from_model(
    container_model: ContainerModel,
    selected_state_names: set[str],
    selected_outcome_instance_ids: set[str],
    selected_text_blocks: list[TextBlock],
) -> None:
    """Remove the selected items from the source model."""

    for state_name in list(selected_state_names):
        container_model.remove_state(state_name)

    removed_outcome_names: set[str] = set()
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

    if isinstance(container_model, Concurrence):
        for outcome_name in removed_outcome_names:
            container_model.outcome_map.pop(outcome_name, None)
