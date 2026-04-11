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
"""Paste helpers for clipboard bundles."""

from __future__ import annotations

import copy

from yasmin_editor.editor_gui.selection_bundle_geometry import get_bundle_bounds
from yasmin_editor.editor_gui.selection_models import ContainerModel, SelectionBundle
from yasmin_editor.editor_gui.selection_names import increment_name
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition


def paste_states(
    target_model: ContainerModel,
    bundle: SelectionBundle,
    offset_x: float,
    offset_y: float,
) -> dict[str, str]:
    """Paste states into the target and return the name remapping."""

    state_name_map: dict[str, str] = {}
    existing_state_names = set(target_model.states.keys())

    for original_name, state in bundle.states.items():
        copied_state = copy.deepcopy(state)
        copied_name = increment_name(copied_state.name, existing_state_names)
        existing_state_names.add(copied_name)
        copied_state.name = copied_name
        target_model.add_state(copied_state)
        state_name_map[original_name] = copied_name

        position = bundle.state_positions.get(original_name)
        if position is not None:
            target_model.layout.set_state_position(
                copied_name,
                position.x + offset_x,
                position.y + offset_y,
            )

    return state_name_map


def paste_outcomes(
    target_model: ContainerModel,
    bundle: SelectionBundle,
    offset_x: float,
    offset_y: float,
) -> tuple[dict[str, str], dict[str, str]]:
    """Paste logical outcomes and their visual aliases into the target."""

    outcome_instance_map: dict[str, str] = {}
    outcome_name_map: dict[str, str] = {}
    existing_outcome_names = {outcome.name for outcome in target_model.outcomes}

    for placement in bundle.outcome_placements:
        copied_name = placement.outcome_name
        outcome_name_map[placement.outcome_name] = copied_name
        if copied_name not in existing_outcome_names:
            outcome_model = copy.deepcopy(bundle.outcomes[placement.outcome_name])
            target_model.add_outcome(outcome_model)
            existing_outcome_names.add(copied_name)
        new_instance_id = target_model.layout.create_outcome_alias(
            copied_name,
            placement.position.x + offset_x,
            placement.position.y + offset_y,
        )
        outcome_instance_map[placement.instance_id] = new_instance_id

    return outcome_name_map, outcome_instance_map


def paste_text_blocks(
    target_model: ContainerModel,
    bundle: SelectionBundle,
    offset_x: float,
    offset_y: float,
) -> None:
    """Paste free text annotations into the target."""

    for text_block in bundle.text_blocks:
        copied_text = copy.deepcopy(text_block)
        copied_text.x += offset_x
        copied_text.y += offset_y
        target_model.add_text_block(copied_text)


def paste_state_machine_links(
    target_model: StateMachine,
    bundle: SelectionBundle,
    state_name_map: dict[str, str],
    outcome_name_map: dict[str, str],
    outcome_instance_map: dict[str, str],
) -> None:
    """Paste transitions that still point to known targets."""

    known_targets = set(state_name_map.values()) | set(outcome_name_map.values())
    known_targets.update(outcome.name for outcome in target_model.outcomes)

    for source_name, transition in zip(bundle.transition_sources, bundle.transitions):
        copied_source = state_name_map.get(source_name)
        if copied_source is None:
            continue
        copied_target = state_name_map.get(transition.target, transition.target)
        if copied_target not in known_targets:
            continue
        target_model.add_transition(
            copied_source,
            Transition(
                source_outcome=transition.source_outcome,
                target=copied_target,
                target_instance_id=outcome_instance_map.get(
                    transition.target_instance_id,
                    "",
                ),
            ),
        )

    if (
        bundle.start_state
        and bundle.start_state in state_name_map
        and not target_model.start_state
    ):
        target_model.start_state = state_name_map[bundle.start_state]


def paste_concurrence_rules(
    target_model: Concurrence,
    bundle: SelectionBundle,
    state_name_map: dict[str, str],
    outcome_name_map: dict[str, str],
) -> None:
    """Paste concurrence outcome-map rules that remain valid after renaming."""

    for outcome_rule in bundle.outcome_rules:
        copied_state_name = state_name_map.get(outcome_rule.state_name)
        copied_outcome_name = outcome_name_map.get(outcome_rule.outcome_name)
        if copied_state_name is None or copied_outcome_name is None:
            continue
        target_model.set_outcome_rule(
            copied_outcome_name,
            copied_state_name,
            outcome_rule.state_outcome,
        )


def paste_bundle_into_model(
    target_model: ContainerModel,
    bundle: SelectionBundle,
    anchor_x: float,
    anchor_y: float,
) -> dict[str, str]:
    """Paste a selection snapshot into a target container model."""

    min_x, min_y, _max_x, _max_y = get_bundle_bounds(bundle)
    offset_x = anchor_x - min_x
    offset_y = anchor_y - min_y

    state_name_map = paste_states(target_model, bundle, offset_x, offset_y)
    outcome_name_map, outcome_instance_map = paste_outcomes(
        target_model,
        bundle,
        offset_x,
        offset_y,
    )
    paste_text_blocks(target_model, bundle, offset_x, offset_y)

    if isinstance(target_model, StateMachine):
        paste_state_machine_links(
            target_model,
            bundle,
            state_name_map,
            outcome_name_map,
            outcome_instance_map,
        )
    elif isinstance(target_model, Concurrence):
        paste_concurrence_rules(
            target_model,
            bundle,
            state_name_map,
            outcome_name_map,
        )

    return state_name_map
