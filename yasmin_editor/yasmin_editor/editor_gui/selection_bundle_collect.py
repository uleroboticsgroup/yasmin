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
"""Selection snapshot builders used by copy, move, and extract actions."""

from __future__ import annotations

import copy
from typing import Iterable

from yasmin_editor.editor_gui.selection_models import (
    ContainerModel,
    OutcomePlacementSnapshot,
    OutcomeRuleSnapshot,
    SelectionBundle,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock


def create_empty_bundle(container_model: ContainerModel) -> SelectionBundle:
    """Create a bundle with the correct source container kind."""

    return SelectionBundle(
        source_kind=(
            "concurrence" if isinstance(container_model, Concurrence) else "state_machine"
        )
    )


def copy_selected_states(
    bundle: SelectionBundle,
    container_model: ContainerModel,
    selected_state_names: set[str],
) -> None:
    """Copy selected states and their layout positions into the bundle."""

    for state_name in selected_state_names:
        state = container_model.states.get(state_name)
        if state is None:
            continue
        bundle.states[state_name] = copy.deepcopy(state)
        position = container_model.layout.get_state_position(state_name)
        if position is not None:
            bundle.state_positions[state_name] = copy.deepcopy(position)


def copy_selected_outcomes(
    bundle: SelectionBundle,
    container_model: ContainerModel,
    selected_outcome_instance_ids: set[str],
) -> set[str]:
    """Copy selected visual outcome aliases and return their logical names."""

    selected_outcome_names: set[str] = set()

    for instance_id in selected_outcome_instance_ids:
        placement = container_model.layout.get_outcome_placement(instance_id)
        if placement is None:
            continue
        selected_outcome_names.add(placement.outcome_name)
        if placement.outcome_name not in bundle.outcomes:
            outcome = next(
                (
                    item
                    for item in container_model.outcomes
                    if item.name == placement.outcome_name
                ),
                None,
            )
            if outcome is not None:
                bundle.outcomes[placement.outcome_name] = copy.deepcopy(outcome)
        bundle.outcome_placements.append(
            OutcomePlacementSnapshot(
                outcome_name=placement.outcome_name,
                instance_id=placement.instance_id,
                position=copy.deepcopy(placement.position),
            )
        )

    return selected_outcome_names


def copy_selected_text_blocks(
    bundle: SelectionBundle,
    selected_text_blocks: Iterable[TextBlock],
) -> None:
    """Copy selected text blocks into the bundle."""

    for text_block in selected_text_blocks:
        bundle.text_blocks.append(copy.deepcopy(text_block))


def copy_state_machine_links(
    bundle: SelectionBundle,
    container_model: StateMachine,
    selected_state_names: set[str],
    selected_outcome_names: set[str],
    selected_outcome_instance_ids: set[str],
) -> None:
    """Copy only transitions that stay inside the selected snapshot."""

    for state_name in selected_state_names:
        for transition in container_model.transitions.get(state_name, []):
            if transition.target in selected_state_names:
                bundle.transition_sources.append(state_name)
                bundle.transitions.append(copy.deepcopy(transition))
                continue
            if transition.target not in selected_outcome_names:
                continue
            if transition.target_instance_id and (
                transition.target_instance_id not in selected_outcome_instance_ids
            ):
                continue
            bundle.transition_sources.append(state_name)
            bundle.transitions.append(copy.deepcopy(transition))

    if container_model.start_state in selected_state_names:
        bundle.start_state = container_model.start_state


def copy_concurrence_rules(
    bundle: SelectionBundle,
    container_model: Concurrence,
    selected_state_names: set[str],
    selected_outcome_names: set[str],
) -> None:
    """Copy concurrence outcome-map rules that stay valid in the bundle."""

    for outcome_name, mapping in container_model.outcome_map.items():
        if outcome_name not in selected_outcome_names:
            continue
        for state_name, state_outcome in mapping.items():
            if state_name not in selected_state_names:
                continue
            bundle.outcome_rules.append(
                OutcomeRuleSnapshot(
                    outcome_name=outcome_name,
                    state_name=state_name,
                    state_outcome=state_outcome,
                )
            )


def collect_selection_bundle(
    container_model: ContainerModel,
    selected_state_names: set[str],
    selected_outcome_instance_ids: set[str],
    selected_text_blocks: list[TextBlock],
) -> SelectionBundle:
    """Build a portable snapshot from the selected model items."""

    bundle = create_empty_bundle(container_model)
    copy_selected_states(bundle, container_model, selected_state_names)
    selected_outcome_names = copy_selected_outcomes(
        bundle,
        container_model,
        selected_outcome_instance_ids,
    )
    copy_selected_text_blocks(bundle, selected_text_blocks)

    if isinstance(container_model, StateMachine):
        copy_state_machine_links(
            bundle,
            container_model,
            selected_state_names,
            selected_outcome_names,
            selected_outcome_instance_ids,
        )
    else:
        copy_concurrence_rules(
            bundle,
            container_model,
            selected_state_names,
            selected_outcome_names,
        )

    return bundle
