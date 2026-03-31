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

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class EditorModelAdapter:
    """Bridge between the editor scene and the pure data model."""

    def __init__(self, editor: "YasminEditor") -> None:
        self.editor = editor

    def create_empty_root_model(self) -> StateMachine:
        return StateMachine(name="")

    def load_from_xml(self, file_path: str) -> StateMachine:
        model = model_from_xml(Path(file_path))
        self.editor.reset_editor_state(model)
        self.editor.set_blackboard_keys(self.editor.keys_to_dicts(model.keys), sync=False)
        self.editor.render_current_container()
        return model

    def save_to_xml(self, file_path: str) -> str:
        self.sync_root_model_from_ui()
        return model_to_xml(self.editor.root_model, Path(file_path))

    def load_external_xml_model(self, file_path: str) -> StateMachine:
        return model_from_xml(Path(file_path))

    def sync_root_model_from_ui(self) -> None:
        self.editor.sync_current_container_layout()

    def rebuild_scene(self, model: StateMachine | Concurrence) -> None:
        self.editor.clear_current_scene()

        for state in model.states.values():
            self.create_state_view(state)

        for outcome in model.outcomes:
            self.create_final_outcome_view(outcome)

        if isinstance(model, StateMachine):
            for owner_name, transitions in model.transitions.items():
                if owner_name not in model.states:
                    continue
                from_view = self.editor.state_nodes.get(owner_name)
                if from_view is None:
                    continue
                for transition in transitions:
                    target_view = self.editor.find_target_view(transition.target)
                    if target_view is None:
                        continue
                    self.editor._create_connection_view(
                        from_view,
                        target_view,
                        transition.source_outcome,
                    )
        else:
            for outcome_name, mapping in model.outcome_map.items():
                to_view = self.editor.final_outcomes.get(outcome_name)
                if to_view is None:
                    continue
                for state_name, source_outcome in mapping.items():
                    from_view = self.editor.state_nodes.get(state_name)
                    if from_view is None:
                        continue
                    self.editor._create_connection_view(
                        from_view, to_view, source_outcome
                    )

    def create_state_view(
        self,
        state_model: State,
        x: float | None = None,
        y: float | None = None,
    ) -> StateNode | ContainerStateNode:
        container_model = self.editor.current_container_model
        pos = container_model.layout.get_state_position(state_model.name)
        if x is None:
            x = pos.x if pos is not None else self.editor.get_free_position().x()
        if y is None:
            y = pos.y if pos is not None else self.editor.get_free_position().y()

        if isinstance(state_model, StateMachine):
            node = ContainerStateNode(
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
            plugin_info = self.editor.resolve_plugin_info_for_model(state_model)
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

        self.editor.canvas.scene.addItem(node)
        self.editor.register_state_node(node)
        return node

    def create_final_outcome_view(
        self,
        outcome_model: Outcome,
        x: float | None = None,
        y: float | None = None,
    ) -> FinalOutcomeNode:
        container_model = self.editor.current_container_model
        pos = container_model.layout.get_outcome_position(outcome_model.name)
        if x is None:
            x = pos.x if pos is not None else 700.0
        if y is None:
            y = pos.y if pos is not None else (len(self.editor.final_outcomes) * 170.0)

        node = FinalOutcomeNode(
            outcome_model.name,
            x,
            y,
            inside_container=True,
            description=outcome_model.description,
            model=outcome_model,
        )
        self.editor.canvas.scene.addItem(node)
        self.editor.final_outcomes[node.name] = node
        return node
