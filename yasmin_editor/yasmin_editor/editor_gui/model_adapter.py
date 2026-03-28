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
from typing import TYPE_CHECKING, Optional

from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition

if TYPE_CHECKING:
    from yasmin_plugins_manager.plugin_info import PluginInfo
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class EditorModelAdapter:
    """Bridge between the editor scene and the pure data model."""

    def __init__(self, editor: "YasminEditor") -> None:
        self.editor = editor

    def create_empty_root_model(self) -> StateMachine:
        return StateMachine(name="")

    def load_from_xml(self, file_path: str) -> StateMachine:
        model = model_from_xml(Path(file_path))
        self.rebuild_scene(model)
        return model

    def save_to_xml(self, file_path: str) -> str:
        self.sync_root_model_from_ui()
        self.sync_all_layouts()
        return model_to_xml(self.editor.root_model, Path(file_path))

    def sync_root_model_from_ui(self) -> None:
        self.editor.root_model.name = self.editor.root_sm_name_edit.text().strip()
        self.editor.root_model.description = (
            self.editor.root_sm_description_edit.text().strip()
        )

    def sync_all_layouts(self) -> None:
        self._sync_container_layout(self.editor.root_model, None)

    def _sync_container_layout(
        self,
        container_model: StateMachine | Concurrence,
        container_view: Optional[ContainerStateNode],
    ) -> None:
        if container_view is None:
            state_views = self.editor.get_root_state_nodes()
            outcome_views = self.editor.final_outcomes
        else:
            state_views = container_view.child_states
            outcome_views = container_view.final_outcomes

        for state_name, state_view in state_views.items():
            container_model.layout.set_state_position(
                state_name,
                float(state_view.pos().x()),
                float(state_view.pos().y()),
            )
            if isinstance(state_view, ContainerStateNode):
                self._sync_container_layout(state_view.model, state_view)

        for outcome_name, outcome_view in outcome_views.items():
            container_model.layout.set_outcome_position(
                outcome_name,
                float(outcome_view.pos().x()),
                float(outcome_view.pos().y()),
            )

    def rebuild_scene(self, model: StateMachine) -> None:
        self.editor.reset_editor_state(model)
        self.editor.root_sm_name_edit.setText(model.name)
        self.editor.root_sm_description_edit.setText(model.description)
        self.editor.set_blackboard_keys(self.editor.keys_to_dicts(model.keys), sync=False)

        for state in model.states.values():
            self.create_state_view(state)

        for outcome in model.outcomes:
            self.create_final_outcome_view(outcome)

        for state in model.states.values():
            if isinstance(state, (StateMachine, Concurrence)):
                self._build_child_views_for_container(self.editor.get_root_state_nodes()[state.name])

        self._build_state_machine_connections(self.editor.root_model, None)

        for state in model.states.values():
            if isinstance(state, (StateMachine, Concurrence)):
                self._build_nested_connections(self.editor.get_root_state_nodes()[state.name])

        self.editor.update_start_state_combo()
        if model.start_state:
            index = self.editor.start_state_combo.findText(model.start_state)
            if index >= 0:
                self.editor.start_state_combo.setCurrentIndex(index)
        self.editor.update_blackboard_usage_highlighting()

    def create_state_view(
        self,
        state_model: State,
        parent_container: Optional[ContainerStateNode] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> StateNode | ContainerStateNode:
        container_model = self.editor.root_model if parent_container is None else parent_container.model

        pos = container_model.layout.get_state_position(state_model.name)
        if x is None:
            x = pos.x if pos is not None else (self.editor.get_free_position().x() if parent_container is None else 0.0)
        if y is None:
            y = pos.y if pos is not None else (self.editor.get_free_position().y() if parent_container is None else 0.0)

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
            node = StateNode(
                state_model.name,
                self.editor.resolve_plugin_info_for_model(state_model),
                x,
                y,
                description=state_model.description,
                defaults=[],
                model=state_model,
            )

        if parent_container is None:
            self.editor.canvas.scene.addItem(node)
        else:
            parent_container.add_child_state(node)
            node.setPos(x, y)

        self.editor.register_state_node(node, parent_container)
        return node

    def create_final_outcome_view(
        self,
        outcome_model: Outcome,
        parent_container: Optional[ContainerStateNode] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> FinalOutcomeNode:
        container_model = self.editor.root_model if parent_container is None else parent_container.model
        pos = container_model.layout.get_outcome_position(outcome_model.name)
        if x is None:
            x = pos.x if pos is not None else (600.0 if parent_container is None else 0.0)
        if y is None:
            y = pos.y if pos is not None else (len(self.editor.final_outcomes) * 150.0 if parent_container is None else 0.0)

        node = FinalOutcomeNode(
            outcome_model.name,
            x,
            y,
            inside_container=parent_container is not None,
            description=outcome_model.description,
            model=outcome_model,
        )

        if parent_container is None:
            self.editor.canvas.scene.addItem(node)
            self.editor.final_outcomes[node.name] = node
        else:
            parent_container.add_final_outcome(node)
            node.setPos(x, y)

        return node

    def _build_child_views_for_container(self, container_view: ContainerStateNode) -> None:
        for state in container_view.model.states.values():
            child_view = self.create_state_view(state, container_view)
            if isinstance(child_view, ContainerStateNode):
                self._build_child_views_for_container(child_view)

        for outcome in container_view.model.outcomes:
            self.create_final_outcome_view(outcome, container_view)

        self._apply_container_layout(container_view)

    def _apply_container_layout(self, container_view: ContainerStateNode) -> None:
        for state_name, child_view in container_view.child_states.items():
            pos = container_view.model.layout.get_state_position(state_name)
            if pos is not None:
                child_view.setPos(pos.x, pos.y)
        for outcome_name, outcome_view in container_view.final_outcomes.items():
            pos = container_view.model.layout.get_outcome_position(outcome_name)
            if pos is not None:
                outcome_view.setPos(pos.x, pos.y)
        container_view.auto_resize_for_children()

    def _build_nested_connections(self, container_view: ContainerStateNode) -> None:
        if isinstance(container_view.model, StateMachine):
            self._build_state_machine_connections(container_view.model, container_view)
        else:
            self._build_concurrence_connections(container_view.model, container_view)

        for child_view in container_view.child_states.values():
            if isinstance(child_view, ContainerStateNode):
                self._build_nested_connections(child_view)

    def _build_state_machine_connections(
        self,
        model: StateMachine,
        container_view: Optional[ContainerStateNode],
    ) -> None:
        for owner_name, transitions in model.transitions.items():
            for transition in transitions:
                from_view = self._resolve_state_machine_source_view(
                    model,
                    container_view,
                    owner_name,
                    transition,
                )
                if from_view is None:
                    continue
                target_view = self.editor.find_target_view(
                    transition.target,
                    getattr(from_view, "parent_container", None),
                )
                if target_view is None:
                    continue
                self.editor._create_connection_view(
                    from_view,
                    target_view,
                    transition.source_outcome,
                )

    def _build_concurrence_connections(
        self,
        model: Concurrence,
        container_view: ContainerStateNode,
    ) -> None:
        for outcome_name, mapping in model.outcome_map.items():
            to_view = container_view.final_outcomes.get(outcome_name)
            if to_view is None:
                continue
            for state_name, source_outcome in mapping.items():
                from_view = container_view.child_states.get(state_name)
                if isinstance(from_view, ContainerStateNode):
                    from_view = from_view.final_outcomes.get(source_outcome, from_view)
                if from_view is None:
                    continue
                self.editor._create_connection_view(from_view, to_view, source_outcome)

    def _resolve_state_machine_source_view(
        self,
        model: StateMachine,
        container_view: Optional[ContainerStateNode],
        owner_name: str,
        transition: Transition,
    ):
        if container_view is None:
            local_states = self.editor.get_root_state_nodes()
            local_outcomes = self.editor.final_outcomes
        else:
            local_states = container_view.child_states
            local_outcomes = container_view.final_outcomes

        from_view = local_states.get(owner_name)
        if isinstance(from_view, ContainerStateNode):
            proxied = from_view.final_outcomes.get(transition.source_outcome)
            if proxied is not None:
                return proxied
            return from_view

        if from_view is not None:
            return from_view

        if owner_name in local_outcomes:
            return local_outcomes[owner_name]

        if owner_name == model.name:
            return local_outcomes.get(transition.source_outcome)

        return None
