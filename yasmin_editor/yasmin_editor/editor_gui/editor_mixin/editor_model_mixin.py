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

from typing import Dict, List, Optional

from PyQt5.QtCore import QPointF
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from yasmin_plugins_manager.plugin_manager import PluginInfo

from yasmin_editor.editor_gui.child_name_conflicts import (
    has_final_outcome_name_conflict as has_child_outcome_name_conflict,
    has_state_name_conflict as has_child_state_name_conflict,
)
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.container_metadata_logic import (
    build_container_metadata_view,
    has_container_name_conflict,
    normalize_container_name,
)
from yasmin_editor.editor_gui.free_position import find_free_position
from yasmin_editor.editor_gui.layout_sync import sync_container_layout_from_views
from yasmin_editor.editor_gui.model_factories import (
    create_container_model as build_container_model,
    create_leaf_model as build_leaf_model,
    resolve_plugin_info_for_model as lookup_plugin_info_for_model,
)
from yasmin_editor.editor_gui.model_parameters import (
    apply_parameter_overwrites as apply_parameter_overwrite_rows,
    dicts_to_parameters as parameter_dicts_to_models,
    get_parameter_overwrites_for_child as build_parameter_overwrite_rows,
    parameters_to_dicts as parameter_models_to_dicts,
)
from yasmin_editor.editor_gui.scene_renderer import create_connection_view
from yasmin_editor.editor_gui.dialogs.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import StateMachineDialog
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import validate_model


class EditorModelMixin:
    """Mixin for editor functionality split from the main window."""

    @staticmethod
    def parameters_to_dicts(parameters: List[Parameter]) -> List[Dict[str, str]]:
        """Return editor-table dictionaries for declared parameters."""

        return parameter_models_to_dicts(parameters)

    @staticmethod
    def dicts_to_parameters(parameters: List[Dict[str, str]]) -> List[Parameter]:
        """Return normalized parameter models from editor-table rows."""

        return parameter_dicts_to_models(parameters)

    def get_parameter_overwrites_for_child(
        self,
        container_model: StateMachine | Concurrence,
        child_model: State,
    ) -> List[Dict[str, str]]:
        """Return one child overwrite table enriched with parent metadata."""

        return build_parameter_overwrite_rows(container_model, child_model)

    def apply_parameter_overwrites(
        self,
        container_model: StateMachine | Concurrence,
        child_model: State,
        overwrites: List[Dict[str, str]],
    ) -> None:
        """Apply one overwrite table back into the container and child models."""

        apply_parameter_overwrite_rows(container_model, child_model, overwrites)

    def resolve_plugin_info_for_model(self, model: State) -> PluginInfo:
        """Resolve the plugin-manager entry that matches one state model."""

        return lookup_plugin_info_for_model(self.plugin_manager, model)

    def create_leaf_model(
        self,
        name: str,
        plugin_info: PluginInfo,
        description: str = "",
        remappings: Optional[Dict[str, str]] = None,
        parameter_mappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
    ) -> State:
        """Create one editor leaf-state model from plugin metadata."""

        return build_leaf_model(
            name,
            plugin_info,
            description=description,
            remappings=remappings,
            parameter_mappings=parameter_mappings,
            outcomes=outcomes,
        )

    def create_container_model(
        self,
        name: str,
        is_concurrence: bool,
        outcomes: Optional[List[str]] = None,
        remappings: Optional[Dict[str, str]] = None,
        start_state: Optional[str] = None,
        default_outcome: Optional[str] = None,
        description: str = "",
        parameter_mappings: Optional[Dict[str, str]] = None,
    ) -> StateMachine | Concurrence:
        """Create one editor container model from dialog input."""

        return build_container_model(
            name,
            is_concurrence=is_concurrence,
            outcomes=outcomes,
            remappings=remappings,
            start_state=start_state,
            default_outcome=default_outcome,
            description=description,
            parameter_mappings=parameter_mappings,
        )

    def _create_connection_view(self, from_node, to_node, outcome: str) -> ConnectionLine:
        return create_connection_view(
            self.canvas.scene,
            self.connections,
            from_node,
            to_node,
            outcome,
        )

    def _rename_state_node_entries(self, old_prefix: str, new_prefix: str) -> None:
        updates = {}
        for key, value in list(self.state_nodes.items()):
            if key == old_prefix or key.startswith(old_prefix + "."):
                suffix = key[len(old_prefix) :]
                updates[new_prefix + suffix] = value
                del self.state_nodes[key]
        self.state_nodes.update(updates)

    def _rename_state_node(self, state_node, new_name: str) -> None:
        old_name = state_node.name
        parent_container = getattr(state_node, "parent_container", None)
        # When editing inside an entered nested container, child nodes do not
        # have a graphical parent container attached. In that case the rename
        # must still be applied to the currently visible container model rather
        # than to the root model.
        parent_model = (
            self.current_container_model
            if parent_container is None
            else parent_container.model
        )
        old_prefix = self.get_state_node_key(old_name, parent_container)
        new_prefix = self.get_state_node_key(new_name, parent_container)

        parent_model.rename_state(old_name, new_name)

        if parent_container is not None:
            parent_container.child_states[new_name] = parent_container.child_states.pop(
                old_name
            )

        self._rename_state_node_entries(old_prefix, new_prefix)
        state_node.name = new_name
        self.update_start_state_combo()

    def apply_common_state_updates(
        self,
        state_node: StateNode | ContainerStateNode,
        new_name: str,
        remappings: Dict[str, str],
        description: str,
        defaults: List[Dict[str, str]],
        parameter_overwrites: Optional[List[Dict[str, str]]] = None,
    ) -> bool:
        if new_name != state_node.name:
            if self.has_state_name_conflict(
                new_name,
                getattr(state_node, "parent_container", None),
            ):
                QMessageBox.warning(
                    self,
                    "Error",
                    f"State '{new_name}' conflicts with an existing state or final outcome in this container!",
                )
                return False
            self._rename_state_node(state_node, new_name)

        state_node.remappings = remappings
        state_node.description = description
        state_node.defaults = defaults
        self.apply_parameter_overwrites(
            self.current_container_model,
            state_node.model,
            list(parameter_overwrites or []),
        )
        return True

    def create_state_node(
        self,
        name: str,
        plugin_info: PluginInfo,
        is_state_machine: bool = False,
        is_concurrence: bool = False,
        outcomes: List[str] = None,
        remappings: Dict[str, str] = None,
        start_state: str = None,
        default_outcome: str = None,
        description: str = "",
        defaults: List[Dict[str, str]] = None,
        parameter_overwrites: List[Dict[str, str]] = None,
    ) -> None:
        """Create a new state node in the canvas."""
        if not name:
            QMessageBox.warning(self, "Validation Error", "Name is required!")
            return

        if self.has_state_name_conflict(name):
            QMessageBox.warning(
                self,
                "Error",
                f"State '{name}' conflicts with an existing state or final outcome in this container!",
            )
            return

        if is_state_machine or is_concurrence:
            model = self.create_container_model(
                name=name,
                is_concurrence=is_concurrence,
                outcomes=outcomes,
                remappings=remappings,
                start_state=start_state,
                default_outcome=default_outcome,
                description=description,
                parameter_mappings={
                    item.get("child_parameter", ""): item.get("name", "")
                    for item in (parameter_overwrites or [])
                    if item.get("child_parameter") and item.get("name")
                },
            )
        else:
            model = self.create_leaf_model(
                name=name,
                plugin_info=plugin_info,
                description=description,
                remappings=remappings,
                parameter_mappings={
                    item.get("child_parameter", ""): item.get("name", "")
                    for item in (parameter_overwrites or [])
                    if item.get("child_parameter") and item.get("name")
                },
                outcomes=outcomes,
            )

        scene_pos = self.canvas.get_preferred_placement_scene_pos()
        node = self.add_model_state(
            model,
            defaults=defaults,
            parameter_overwrites=parameter_overwrites,
            x=float(scene_pos.x()),
            y=float(scene_pos.y()),
        )
        self.start_pending_node_placement(node)
        self.record_history_checkpoint()

    def add_container(self, is_concurrence: bool = False) -> None:
        """Add a new container (State Machine or Concurrence)."""
        dialog = (
            ConcurrenceDialog(parent=self)
            if is_concurrence
            else StateMachineDialog(parent=self)
        )
        if dialog.exec_():
            result = (
                dialog.get_concurrence_data()
                if is_concurrence
                else dialog.get_state_machine_data()
            )
            if result:
                name, outcomes, param, remappings, description, defaults = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=not is_concurrence,
                    is_concurrence=is_concurrence,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=param if not is_concurrence else None,
                    default_outcome=param if is_concurrence else None,
                    description=description,
                    defaults=defaults,
                )

    @property
    def current_container_model(self):
        """Return the model of the container currently shown in the canvas."""
        return self.current_container_path[-1]

    @property
    def current_parent_model(self):
        """Return the parent model of the currently shown container, if any."""
        if len(self.current_container_path) < 2:
            return None
        return self.current_container_path[-2]

    @property
    def start_state(self) -> Optional[str]:
        """Return the start state of the current state machine container."""
        model = self.current_container_model
        return model.start_state if isinstance(model, StateMachine) else None

    @start_state.setter
    def start_state(self, value: Optional[str]) -> None:
        """Update the start state of the current state machine container."""
        model = self.current_container_model
        if isinstance(model, StateMachine):
            model.start_state = value

    @property
    def default_outcome(self) -> Optional[str]:
        """Return the default outcome of the current concurrence container."""
        model = self.current_container_model
        return model.default_outcome if isinstance(model, Concurrence) else None

    @default_outcome.setter
    def default_outcome(self, value: Optional[str]) -> None:
        """Update the default outcome of the current concurrence container."""
        model = self.current_container_model
        if isinstance(model, Concurrence):
            model.default_outcome = value

    def clear_current_scene(self) -> None:
        self._reset_pending_selection_state()
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()
        self.text_blocks.clear()

    def get_final_outcome_views(
        self, outcome_name: str | None = None
    ) -> List[FinalOutcomeNode]:
        views = list(self.final_outcomes.values())
        if outcome_name is not None:
            views = [view for view in views if view.name == outcome_name]
        return views

    def get_primary_final_outcome_view(
        self, outcome_name: str
    ) -> Optional[FinalOutcomeNode]:
        views = self.get_final_outcome_views(outcome_name)
        return views[0] if views else None

    def sync_current_container_layout(self) -> None:
        sync_container_layout_from_views(
            self.current_container_model,
            self.state_nodes,
            self.final_outcomes.values(),
            self.text_blocks,
        )

    def update_container_controls(self) -> None:
        if not hasattr(self, "root_sm_name_edit"):
            return
        model = self.current_container_model
        metadata_view = build_container_metadata_view(model)
        self.root_sm_name_edit.blockSignals(True)
        self.root_sm_description_edit.blockSignals(True)
        self.start_state_combo.blockSignals(True)

        self.root_sm_name_label.setText(metadata_view.name_label_html)
        self.start_state_label.setText(metadata_view.selector_label_html)

        self.root_sm_name_edit.setText(model.name)
        self.root_sm_description_edit.setText(model.description)

        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")
        for item_name in metadata_view.selector_items:
            self.start_state_combo.addItem(item_name)

        if metadata_view.current_selector_value:
            index = self.start_state_combo.findText(metadata_view.current_selector_value)
            self.start_state_combo.setCurrentIndex(index if index >= 0 else 0)
        else:
            self.start_state_combo.setCurrentIndex(0)

        self.root_sm_name_edit.blockSignals(False)
        self.root_sm_description_edit.blockSignals(False)
        self.start_state_combo.blockSignals(False)

    def on_root_sm_name_changed(self, text: str) -> None:
        model = self.current_container_model
        parent_model = self.current_parent_model
        text = normalize_container_name(text)
        if parent_model is None:
            old_name = model.name
            model.name = text
            if isinstance(model, StateMachine):
                model.rename_transition_owner(old_name, text)
        else:
            old_name = model.name
            if has_container_name_conflict(
                text,
                current_name=old_name,
                sibling_state_names=parent_model.states.keys(),
                sibling_outcome_names=[outcome.name for outcome in parent_model.outcomes],
            ):
                return
            parent_model.rename_state(old_name, text)
        self.refresh_breadcrumbs()
        self.record_history_checkpoint()

    def on_root_sm_description_changed(self, text: str) -> None:
        self.current_container_model.description = text
        self.record_history_checkpoint()

    def on_start_state_changed(self, text: str) -> None:
        if isinstance(self.current_container_model, StateMachine):
            self.start_state = None if text == "(None)" else text
        else:
            self.default_outcome = None if text == "(None)" else text
        self.refresh_start_state_indicators()
        self.record_history_checkpoint()

    def refresh_start_state_indicators(self) -> None:
        start_state_name = None
        if isinstance(self.current_container_model, StateMachine):
            start_state_name = self.current_container_model.start_state

        for state_name, node in self.state_nodes.items():
            if not hasattr(node, "set_start_indicator"):
                continue
            is_start_state = bool(start_state_name and state_name == start_state_name)
            node.set_start_indicator(
                is_start_state,
                f"Start state: {start_state_name}" if start_state_name else "",
            )

    def update_start_state_combo(self) -> None:
        self.update_container_controls()
        self.refresh_start_state_indicators()

    def reset_editor_state(self, model: Optional[StateMachine] = None) -> None:
        self.clear_current_scene()
        self._blackboard_keys = []
        self._blackboard_key_metadata = {}
        self.root_model = model or self.model_adapter.create_empty_root_model()
        self.current_container_path = [self.root_model]
        self.current_file_path = None
        self.current_runtime_container_path = []
        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self.runtime_breakpoints_before = set()
        self._delete_runtime_snapshot()
        self.clear_runtime_log_view()
        self.refresh_blackboard_keys_list()
        self.update_container_controls()
        self.refresh_breadcrumbs()

    def save_to_xml(self, file_path: str) -> None:
        self.model_adapter.save_to_xml(file_path)

    def get_root_state_nodes(self) -> Dict[str, StateNode | ContainerStateNode]:
        return self.state_nodes

    def get_container_path(self, container: Optional[ContainerStateNode]) -> str:
        if container is None:
            return ""
        return container.name

    def get_state_node_key(
        self,
        name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        return name

    def register_state_node(
        self,
        state_node: StateNode | ContainerStateNode,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        self.state_nodes[state_node.name] = state_node
        return state_node.name

    def find_target_view(
        self,
        target_name: str,
        source_container=None,
        target_instance_id: str = "",
    ):
        target = self.state_nodes.get(target_name)
        if target is not None:
            return target
        if target_instance_id:
            instance_view = self.final_outcomes.get(target_instance_id)
            if instance_view is not None:
                return instance_view
        return self.get_primary_final_outcome_view(target_name)

    def has_state_name_conflict(
        self,
        state_name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> bool:
        container_model = (
            self.current_container_model
            if parent_container is None
            else parent_container.model
        )
        return has_child_state_name_conflict(
            state_name,
            sibling_state_names=container_model.states.keys(),
            sibling_outcome_names=[outcome.name for outcome in container_model.outcomes],
        )

    def has_final_outcome_name_conflict(
        self,
        outcome_name: str,
        current_name: str | None = None,
    ) -> bool:
        return has_child_outcome_name_conflict(
            outcome_name,
            current_name=current_name,
            sibling_state_names=self.current_container_model.states.keys(),
            sibling_outcome_names=[
                outcome.name for outcome in self.current_container_model.outcomes
            ],
        )

    def add_model_state(
        self,
        model: State,
        defaults: Optional[List[Dict[str, str]]] = None,
        parent_container: Optional[ContainerStateNode] = None,
        parameter_overwrites: Optional[List[Dict[str, str]]] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> StateNode | ContainerStateNode:
        position = self.get_free_position()
        x = position.x() if x is None else x
        y = position.y() if y is None else y

        container_model = self.current_container_model
        container_model.add_state(model)
        self.apply_parameter_overwrites(
            container_model,
            model,
            list(parameter_overwrites or []),
        )
        container_model.layout.set_state_position(model.name, x, y)
        node = self.model_adapter.create_state_view(model, x=x, y=y)
        node.defaults = defaults or []

        if isinstance(container_model, StateMachine):
            if len(container_model.states) == 1 and not container_model.start_state:
                container_model.start_state = model.name
        self.update_start_state_combo()
        self.sync_blackboard_keys()
        self.refresh_connection_port_visibility()
        return node

    def start_pending_node_placement(
        self,
        node: StateNode | ContainerStateNode | FinalOutcomeNode | TextBlockNode,
    ) -> None:
        existing_pending = self.canvas.pending_placement_item
        if existing_pending is not None and existing_pending is not node:
            self.cancel_pending_node_placement(existing_pending)
        self.canvas.start_pending_placement(node)

    def finalize_pending_node_placement(
        self,
        node: StateNode | ContainerStateNode | FinalOutcomeNode | TextBlockNode,
        scene_pos: QPointF,
    ) -> None:
        node.setPos(scene_pos)

        if isinstance(node, FinalOutcomeNode):
            self.current_container_model.layout.set_outcome_position(
                node.name,
                float(scene_pos.x()),
                float(scene_pos.y()),
                instance_id=node.instance_id or None,
            )
            self.statusBar().showMessage(f"Added final outcome: {node.name}", 2000)
        elif isinstance(node, TextBlockNode):
            node.model.x = float(scene_pos.x())
            node.model.y = float(scene_pos.y())
            self.statusBar().showMessage("Added text block", 2000)
        else:
            self.current_container_model.layout.set_state_position(
                node.name,
                float(scene_pos.x()),
                float(scene_pos.y()),
            )
            self.statusBar().showMessage(f"Added state: {node.name}", 2000)

        self.canvas.clear_pending_placement()
        self.canvas.scene.clearSelection()
        node.setSelected(True)
        self.sync_current_container_layout()

        if isinstance(node, TextBlockNode):
            node.enter_edit_mode()

        self.record_history_checkpoint()

    def cancel_pending_node_placement(
        self,
        node: StateNode | ContainerStateNode | FinalOutcomeNode | TextBlockNode,
    ) -> None:
        if isinstance(node, FinalOutcomeNode):
            siblings = [
                view
                for view in self.get_final_outcome_views(node.name)
                if view is not node
            ]
            if node.instance_id:
                self.current_container_model.layout.remove_outcome_placement(
                    node.instance_id
                )
                self.final_outcomes.pop(node.instance_id, None)
            if not siblings:
                self.current_container_model.remove_outcome(node.name)
            message = f"Canceled final outcome: {node.name}"
        elif isinstance(node, TextBlockNode):
            self.current_container_model.remove_text_block(node.model)
            if node in self.text_blocks:
                self.text_blocks.remove(node)
            message = "Canceled text block"
        else:
            self.current_container_model.remove_state(node.name)
            self.state_nodes.pop(node.name, None)
            self.sync_blackboard_keys()
            message = f"Canceled state: {node.name}"

        self.canvas.scene.removeItem(node)
        self.canvas.clear_pending_placement()
        self.update_start_state_combo()
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(message, 2000)
        self.record_history_checkpoint()

    def rename_final_outcome(
        self,
        outcome_node: FinalOutcomeNode,
        new_name: str,
    ) -> None:
        """Rename a logical final outcome and refresh every visible alias."""

        old_name = outcome_node.name
        if old_name == new_name:
            return

        views = self.get_final_outcome_views(old_name)
        container_model = self.current_container_model
        container_model.rename_outcome(old_name, new_name)

        for outcome_view in views:
            outcome_view.name = new_name
            outcome_view.update_tooltip()
            outcome_view.update_attached_connections()

        parent_model = self.current_parent_model
        if parent_model is not None:
            parent_model.rename_child_state_outcome(
                container_model.name,
                old_name,
                new_name,
            )

        self.update_start_state_combo()
        self.refresh_connection_port_visibility()
        self.record_history_checkpoint()

    def register_connection_in_model(self, from_node, to_node, outcome: str) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.set_outcome_rule(to_node.name, from_node.name, outcome)
            return
        owner_model.add_transition(
            from_node.name,
            Transition(
                source_outcome=outcome,
                target=to_node.name,
                target_instance_id=getattr(to_node, "instance_id", ""),
            ),
        )

    def unregister_connection_in_model(self, connection: ConnectionLine) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.remove_outcome_rule(
                connection.to_node.name,
                connection.from_node.name,
                connection.outcome,
            )
            return
        owner_model.remove_transition(
            connection.from_node.name,
            connection.outcome,
            connection.to_node.name,
        )

    def delete_state_item(self, state_node: StateNode | ContainerStateNode) -> None:
        for connection in list(state_node.connections):
            self.delete_connection_item(connection)

        self.state_nodes.pop(state_node.name, None)
        self.current_container_model.remove_state(state_node.name)
        self.canvas.scene.removeItem(state_node)
        self.update_start_state_combo()
        self.sync_blackboard_keys()
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(f"Deleted state: {state_node.name}", 2000)

    def delete_final_outcome_item(self, outcome_node: FinalOutcomeNode) -> None:
        incoming_connections = [
            connection
            for connection in list(outcome_node.connections)
            if connection.to_node == outcome_node
        ]
        sibling_views = [
            view
            for view in self.get_final_outcome_views(outcome_node.name)
            if view is not outcome_node
        ]

        if sibling_views:
            replacement_view = sibling_views[0]
            for connection in incoming_connections:
                old_target = connection.to_node
                if old_target is not None:
                    old_target.remove_connection(connection)
                connection.to_node = replacement_view
                replacement_view.add_connection(connection)
                connection.update_position()
                if isinstance(self.current_container_model, StateMachine):
                    owner_transitions = self.current_container_model.transitions.get(
                        connection.from_node.name, []
                    )
                    for transition in owner_transitions:
                        if (
                            transition.source_outcome == connection.outcome
                            and transition.target == outcome_node.name
                            and transition.target_instance_id == outcome_node.instance_id
                        ):
                            transition.target_instance_id = replacement_view.instance_id
                            break
            if outcome_node.instance_id:
                self.current_container_model.layout.remove_outcome_placement(
                    outcome_node.instance_id
                )
                self.final_outcomes.pop(outcome_node.instance_id, None)
            self.canvas.scene.removeItem(outcome_node)
            self.refresh_connection_port_visibility()
            self.statusBar().showMessage(
                f"Deleted final outcome view: {outcome_node.name}",
                2000,
            )
            return

        for connection in list(outcome_node.connections):
            self.delete_connection_item(connection)

        self.final_outcomes.pop(outcome_node.instance_id, None)
        self.current_container_model.remove_outcome(outcome_node.name)
        self.canvas.scene.removeItem(outcome_node)
        self.update_start_state_combo()
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(
            f"Deleted final outcome: {outcome_node.name}",
            2000,
        )

    def delete_text_block_item(self, text_block_node: TextBlockNode) -> None:
        """Delete one free-form text block from the current container."""
        self.current_container_model.remove_text_block(text_block_node.model)
        if text_block_node in self.text_blocks:
            self.text_blocks.remove(text_block_node)
        self.canvas.scene.removeItem(text_block_node)
        self.statusBar().showMessage("Deleted text block", 2000)

    def add_text_block_model(
        self,
        content: str = "# Notes\n",
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> TextBlockNode:
        """Create a new inline-editable text block in the current container."""
        position = self.get_free_position()
        x = position.x() if x is None else x
        y = position.y() if y is None else y

        model = TextBlock(x=float(x), y=float(y), content=content)
        self.current_container_model.add_text_block(model)
        node = self.model_adapter.create_text_block_view(model, x=float(x), y=float(y))
        return node

    def add_state_to_container(self) -> None:
        self.add_state()

    def add_state_machine_to_container(self) -> None:
        self.add_state_machine()

    def add_concurrence_to_container(self) -> None:
        self.add_concurrence()

    def _confirm_save_despite_validation_errors(self) -> bool:
        """Ask the user whether the state machine should still be saved."""

        validation = validate_model(self.root_model)
        errors = [f"- {item.message}" for item in validation.errors]
        if not errors:
            return True

        error_msg = (
            "Cannot save state machine. Please fix the following issues:\n\n"
            + "\n".join(errors)
        )
        reply = QMessageBox.critical(
            self,
            "Validation Errors",
            error_msg + "\n\nDo you want to save anyway?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return reply == QMessageBox.Yes

    def _normalize_save_file_path(self, file_path: str) -> str:
        """Return the normalized XML save path for the document."""

        return file_path if file_path.lower().endswith(".xml") else f"{file_path}.xml"

    def _select_state_machine_save_path(self) -> Optional[str]:
        """Prompt the user for an XML save path."""

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save State Machine", self.current_file_path or "", "XML Files (*.xml)"
        )
        if not file_path:
            return None
        return self._normalize_save_file_path(file_path)

    def _save_state_machine_to_path(self, file_path: str) -> bool:
        """Save the current state machine to one concrete file path."""

        self.sync_current_container_layout()
        if not self._confirm_save_despite_validation_errors():
            return False

        try:
            self.save_to_xml(file_path)
            self.current_file_path = file_path
            self.register_recent_file(file_path)
            self.reset_document_dirty_state()
            self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            return True
        except Exception as error:
            QMessageBox.critical(self, "Error", f"Failed to save file: {str(error)}")
            return False

    def save_state_machine(self) -> bool:
        """Save the current state machine to the active document path."""

        if self.current_file_path:
            return self._save_state_machine_to_path(self.current_file_path)
        return self.save_state_machine_as()

    def save_state_machine_as(self) -> bool:
        """Prompt for a target file path and save the current state machine."""

        file_path = self._select_state_machine_save_path()
        if file_path is None:
            return False
        return self._save_state_machine_to_path(file_path)

    def get_free_position(self) -> QPointF:
        """Get a free position close to the current viewport center."""
        if not hasattr(self, "canvas"):
            return QPointF(100, 100)

        viewport_rect = self.canvas.viewport().rect()
        visible_rect = self.canvas.mapToScene(viewport_rect).boundingRect()
        center = (visible_rect.center().x(), visible_rect.center().y())
        occupied_positions = [
            (item.pos().x(), item.pos().y())
            for item in list(self.state_nodes.values())
            + list(self.final_outcomes.values())
            + list(self.text_blocks)
        ]
        position = find_free_position(center, occupied_positions)
        return QPointF(position[0], position[1])

    def load_from_xml(self, file_path: str) -> None:
        self.model_adapter.load_from_xml(file_path)
