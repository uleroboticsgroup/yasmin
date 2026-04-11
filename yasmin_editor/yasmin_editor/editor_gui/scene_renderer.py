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
"""Shared scene building helpers for the editor canvases."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol

from PyQt5.QtWidgets import QGraphicsScene

from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.layout import OutcomePlacement
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock


class _PluginInfoLike(Protocol):
    plugin_type: str | None
    outcomes: list[str] | None


SceneStateNode = StateNode | ContainerStateNode
SceneTargetView = SceneStateNode | FinalOutcomeNode
ContainerModel = StateMachine | Concurrence


@dataclass(slots=True)
class SceneRenderContext:
    """Scene-specific hooks and registries used during rendering."""

    scene: QGraphicsScene
    state_nodes: dict[str, SceneStateNode]
    final_outcomes: dict[str, FinalOutcomeNode]
    connections: list[ConnectionLine]
    text_blocks: list[TextBlockNode]
    clear_scene: Callable[[], None]
    register_state_node: Callable[[SceneStateNode], None]
    resolve_plugin_info: Callable[[State], _PluginInfoLike | None]
    resolve_target_view: Callable[[str, str], SceneTargetView | None]
    resolve_primary_outcome_view: Callable[[str], FinalOutcomeNode | None]
    create_connection_view: Callable[
        [SceneTargetView, SceneTargetView, str], ConnectionLine
    ]
    ensure_outcome_placements: Callable[
        [ContainerModel, Outcome, int], list[OutcomePlacement]
    ]


def create_connection_view(
    scene: QGraphicsScene,
    connections: list[ConnectionLine],
    from_node: SceneTargetView,
    to_node: SceneTargetView,
    outcome: str,
) -> ConnectionLine:
    """Create a connection item and register all of its scene fragments."""

    connection = ConnectionLine(from_node, to_node, outcome)
    scene.addItem(connection)
    scene.addItem(connection.arrow_head)
    scene.addItem(connection.label_bg)
    scene.addItem(connection.label)
    connections.append(connection)
    for existing_connection in connections:
        existing_connection.update_position()
    return connection


def create_text_block_view(
    scene: QGraphicsScene,
    text_block_model: TextBlock,
    *,
    read_only: bool,
    x: float | None = None,
    y: float | None = None,
) -> TextBlockNode:
    """Create one text-block node from its model."""

    node = TextBlockNode(
        x=float(text_block_model.x if x is None else x),
        y=float(text_block_model.y if y is None else y),
        content=text_block_model.content,
        model=text_block_model,
    )
    node.set_read_only(read_only)
    scene.addItem(node)
    return node


def create_state_view(
    scene: QGraphicsScene,
    state_model: State,
    *,
    resolve_plugin_info: Callable[[State], _PluginInfoLike | None],
    x: float,
    y: float,
) -> SceneStateNode:
    """Create one state node from its model."""

    if isinstance(state_model, StateMachine):
        node: SceneStateNode = ContainerStateNode(
            state_model.name,
            x,
            y,
            False,
            description=state_model.description,
            defaults=[],
            model=state_model,
        )
    elif isinstance(state_model, Concurrence):
        node = ContainerStateNode(
            state_model.name,
            x,
            y,
            True,
            description=state_model.description,
            defaults=[],
            model=state_model,
        )
    else:
        plugin_info = resolve_plugin_info(state_model)
        if not state_model.outcomes:
            for outcome_name in list(getattr(plugin_info, "outcomes", []) or []):
                state_model.add_outcome(Outcome(name=outcome_name))
        if state_model.state_type == "xml":
            node = ContainerStateNode(
                state_model.name,
                x,
                y,
                False,
                description=state_model.description,
                defaults=[],
                model=state_model,
                state_kind_label="XML",
                is_xml_reference=True,
            )
            node.plugin_info = plugin_info
        else:
            node = StateNode(
                state_model.name,
                plugin_info,
                x,
                y,
                description=state_model.description,
                defaults=[],
                model=state_model,
            )

    scene.addItem(node)
    return node


def create_final_outcome_view(
    scene: QGraphicsScene,
    outcome_model: Outcome,
    placement: OutcomePlacement,
) -> FinalOutcomeNode:
    """Create one visible final-outcome alias from its placement model."""

    node = FinalOutcomeNode(
        outcome_model.name,
        placement.position.x,
        placement.position.y,
        inside_container=True,
        description=outcome_model.description,
        model=outcome_model,
        instance_id=placement.instance_id,
    )
    scene.addItem(node)
    return node


def ensure_outcome_placements(
    model: ContainerModel,
    outcome_model: Outcome,
    fallback_index: int,
) -> list[OutcomePlacement]:
    """Return visible placements for one final outcome, creating a default alias if needed."""

    placements = model.layout.get_outcome_placements(outcome_model.name)
    if placements:
        return placements

    pos = model.layout.get_outcome_position(outcome_model.name)
    x = pos.x if pos is not None else 700.0
    y = pos.y if pos is not None else float(fallback_index * 170.0)
    instance_id = model.layout.create_outcome_alias(
        outcome_model.name,
        float(x),
        float(y),
    )
    placement = model.layout.get_outcome_placement(instance_id)
    return [placement] if placement is not None else []


def render_container_scene(
    model: ContainerModel,
    context: SceneRenderContext,
) -> None:
    """Rebuild a canvas scene from one container model."""

    context.clear_scene()

    for text_block in model.text_blocks:
        node = create_text_block_view(context.scene, text_block, read_only=False)
        context.text_blocks.append(node)

    for state in model.states.values():
        position = model.layout.get_state_position(state.name)
        x = position.x if position is not None else 0.0
        y = position.y if position is not None else 0.0
        node = create_state_view(
            context.scene,
            state,
            resolve_plugin_info=context.resolve_plugin_info,
            x=x,
            y=y,
        )
        context.register_state_node(node)

    for outcome in model.outcomes:
        placements = context.ensure_outcome_placements(
            model,
            outcome,
            len(context.final_outcomes),
        )
        for placement in placements:
            node = create_final_outcome_view(context.scene, outcome, placement)
            context.final_outcomes[node.instance_id] = node

    if isinstance(model, StateMachine):
        for owner_name, transitions in model.transitions.items():
            from_view = context.state_nodes.get(owner_name)
            if from_view is None:
                continue
            for transition in transitions:
                to_view = context.resolve_target_view(
                    transition.target,
                    transition.target_instance_id,
                )
                if to_view is None:
                    continue
                context.create_connection_view(
                    from_view,
                    to_view,
                    transition.source_outcome,
                )
        return

    for outcome_name, mapping in model.outcome_map.items():
        to_view = context.resolve_primary_outcome_view(outcome_name)
        if to_view is None:
            continue
        for state_name, source_outcome in mapping.items():
            from_view = context.state_nodes.get(state_name)
            if from_view is None:
                continue
            context.create_connection_view(from_view, to_view, source_outcome)
