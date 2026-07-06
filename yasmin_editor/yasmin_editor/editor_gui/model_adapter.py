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

from pathlib import Path
from typing import TYPE_CHECKING, List, Union

from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.editor_gui.scene_renderer import (
    SceneRenderContext,
    create_final_outcome_view,
    create_state_view,
    create_text_block_view,
    ensure_outcome_placements,
    render_container_scene,
)
from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.layout import OutcomePlacement, Position
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class EditorModelAdapter:
    """Bridge between the editor scene and the pure data model."""

    def __init__(self, editor: "YasminEditor") -> None:
        self.editor = editor

    def create_empty_root_model(self) -> StateMachine:
        return StateMachine(name="")

    def apply_root_model(
        self,
        model: StateMachine,
        *,
        current_file_path: Union[str, None] = None,
        fit_view: bool = False,
        reset_history: bool = True,
    ) -> StateMachine:
        """Replace the current editor contents with the provided root model."""

        self.editor.reset_editor_state(model)
        self.editor.set_blackboard_keys(self.editor.keys_to_dicts(model.keys), sync=False)
        self.editor.current_file_path = current_file_path
        self.editor.render_current_container(fit_view=fit_view)
        if reset_history:
            self.editor.reset_history()
        return model

    def load_from_xml(self, file_path: str) -> StateMachine:
        model = model_from_xml(Path(file_path))
        return self.apply_root_model(
            model,
            current_file_path=file_path,
            fit_view=True,
        )

    def save_to_xml(self, file_path: str) -> str:
        self.sync_root_model_from_ui()
        return model_to_xml(self.editor.root_model, Path(file_path))

    def load_external_xml_model(self, file_path: str) -> StateMachine:
        return model_from_xml(Path(file_path))

    def sync_root_model_from_ui(self) -> None:
        self.editor.sync_current_container_layout()

    def _build_main_scene_context(self) -> SceneRenderContext:
        return SceneRenderContext(
            scene=self.editor.canvas.scene,
            state_nodes=self.editor.state_nodes,
            final_outcomes=self.editor.final_outcomes,
            connections=self.editor.connections,
            text_blocks=self.editor.text_blocks,
            clear_scene=self.editor.clear_current_scene,
            register_state_node=lambda node: self.editor.register_state_node(node),
            resolve_plugin_info=self.editor.resolve_plugin_info_for_model,
            resolve_target_view=lambda target_name, instance_id: self.editor.find_target_view(
                target_name,
                target_instance_id=instance_id,
            ),
            resolve_primary_outcome_view=self.editor.get_primary_final_outcome_view,
            create_connection_view=self.editor._create_connection_view,
            ensure_outcome_placements=ensure_outcome_placements,
        )

    def rebuild_scene(
        self, model: Union[StateMachine, Concurrence, OrthogonalState]
    ) -> None:
        render_container_scene(model, self._build_main_scene_context())

    def create_state_view(
        self,
        state_model: State,
        x: Union[float, None] = None,
        y: Union[float, None] = None,
    ) -> Union[StateNode, ContainerStateNode]:
        container_model = self.editor.current_container_model
        pos = container_model.layout.get_state_position(state_model.name)
        if x is None:
            x = pos.x if pos is not None else self.editor.get_free_position().x()
        if y is None:
            y = pos.y if pos is not None else self.editor.get_free_position().y()

        node = create_state_view(
            self.editor.canvas.scene,
            state_model,
            resolve_plugin_info=self.editor.resolve_plugin_info_for_model,
            x=x,
            y=y,
        )
        self.editor.register_state_node(node)
        return node

    def create_text_block_view(
        self,
        text_block_model: TextBlock,
        x: Union[float, None] = None,
        y: Union[float, None] = None,
    ) -> TextBlockNode:
        node = create_text_block_view(
            self.editor.canvas.scene,
            text_block_model,
            read_only=False,
            x=x,
            y=y,
        )
        self.editor.text_blocks.append(node)
        return node

    def create_final_outcome_view(
        self,
        outcome_model: Outcome,
        x: float,
        y: float,
        instance_id: str,
    ) -> FinalOutcomeNode:
        placement = OutcomePlacement(
            instance_id=instance_id,
            outcome_name=outcome_model.name,
            position=Position(x, y),
        )
        node = create_final_outcome_view(
            self.editor.canvas.scene, outcome_model, placement
        )
        self.editor.final_outcomes[node.instance_id] = node
        return node

    def create_final_outcome_views(
        self,
        outcome_model: Outcome,
        x: Union[float, None] = None,
        y: Union[float, None] = None,
    ) -> List[FinalOutcomeNode]:
        container_model = self.editor.current_container_model
        placements = container_model.layout.get_outcome_placements(outcome_model.name)
        if not placements:
            pos = container_model.layout.get_outcome_position(outcome_model.name)
            if x is None:
                x = pos.x if pos is not None else 700.0
            if y is None:
                y = (
                    pos.y
                    if pos is not None
                    else (len(self.editor.final_outcomes) * 170.0)
                )
            instance_id = container_model.layout.create_outcome_alias(
                outcome_model.name,
                float(x),
                float(y),
            )
            placement = container_model.layout.get_outcome_placement(instance_id)
            placements = [placement] if placement is not None else []

        created_nodes: List[FinalOutcomeNode] = []
        for placement in placements:
            if placement is None or placement.instance_id in self.editor.final_outcomes:
                continue
            node = create_final_outcome_view(
                self.editor.canvas.scene,
                outcome_model,
                placement,
            )
            self.editor.final_outcomes[node.instance_id] = node
            created_nodes.append(node)
        return created_nodes
