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

from yasmin_editor.editor_gui.connection_line import ConnectionLine
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
        return [
            {
                "name": parameter.name,
                "description": parameter.description,
                "default_type": parameter.default_type,
                "default_value": parameter.default_value,
                "has_default": parameter.has_default,
            }
            for parameter in parameters
        ]

    @staticmethod
    def dicts_to_parameters(parameters: List[Dict[str, str]]) -> List[Parameter]:
        normalized: List[Parameter] = []
        for item in parameters:
            name = str(item.get("name", "") or "").strip()
            if not name:
                continue
            normalized.append(
                Parameter(
                    name=name,
                    description=str(item.get("description", "") or "").strip(),
                    default_type=str(item.get("default_type", "") or "").strip(),
                    default_value=item.get("default_value"),
                )
            )
        return normalized

    def get_parameter_overwrites_for_child(
        self,
        container_model: StateMachine | Concurrence,
        child_model: State,
    ) -> List[Dict[str, str]]:
        declared_by_name = {
            parameter.name: parameter for parameter in container_model.parameters
        }
        overwrites: List[Dict[str, str]] = []
        for child_parameter, parent_parameter in child_model.parameter_mappings.items():
            declared = declared_by_name.get(parent_parameter)
            overwrites.append(
                {
                    "name": parent_parameter,
                    "child_parameter": child_parameter,
                    "description": getattr(declared, "description", ""),
                    "default_type": getattr(declared, "default_type", ""),
                    "default_value": getattr(declared, "default_value", ""),
                }
            )
        return overwrites

    def apply_parameter_overwrites(
        self,
        container_model: StateMachine | Concurrence,
        child_model: State,
        overwrites: List[Dict[str, str]],
    ) -> None:
        child_model.parameter_mappings.clear()
        for item in overwrites:
            child_parameter = str(item.get("child_parameter", "") or "").strip()
            parent_parameter = str(item.get("name", "") or "").strip()
            if child_parameter and parent_parameter:
                child_model.parameter_mappings[child_parameter] = parent_parameter

        declarations_by_name = {
            parameter.name: parameter for parameter in container_model.parameters
        }
        declaration_order = [parameter.name for parameter in container_model.parameters]

        for item in overwrites:
            name = str(item.get("name", "") or "").strip()
            if not name:
                continue
            declarations_by_name[name] = Parameter(
                name=name,
                description=str(item.get("description", "") or "").strip(),
                default_type=str(item.get("default_type", "") or "").strip(),
                default_value=item.get("default_value"),
            )
            if name not in declaration_order:
                declaration_order.append(name)

        used_names = {
            parent_name
            for state in container_model.states.values()
            for parent_name in state.parameter_mappings.values()
        }

        container_model.parameters = [
            declarations_by_name[name]
            for name in declaration_order
            if name in used_names and name in declarations_by_name
        ]

    def resolve_plugin_info_for_model(self, model: State) -> PluginInfo:
        if model.state_type == "py":
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.python_plugins
                    if item.module == model.module and item.class_name == model.class_name
                ),
                None,
            )
        elif model.state_type == "cpp":
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.cpp_plugins
                    if item.class_name == model.class_name
                ),
                None,
            )
        else:
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.xml_files
                    if item.file_name == model.file_name
                    and (
                        not model.package_name or item.package_name == model.package_name
                    )
                ),
                None,
            )
            if plugin is None and model.file_name:
                plugin = next(
                    (
                        item
                        for item in self.plugin_manager.xml_files
                        if item.file_name == model.file_name
                    ),
                    None,
                )

        if plugin is None:
            raise ValueError(f"Unable to resolve plugin for state '{model.name}'")
        return plugin

    def create_leaf_model(
        self,
        name: str,
        plugin_info: PluginInfo,
        description: str = "",
        remappings: Optional[Dict[str, str]] = None,
        parameter_mappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
    ) -> State:
        state_type = {"python": "py", "cpp": "cpp", "xml": "xml"}.get(
            plugin_info.plugin_type,
            plugin_info.plugin_type,
        )
        model = State(
            name=name,
            description=description or "",
            remappings=dict(remappings or {}),
            parameter_mappings=dict(parameter_mappings or {}),
            state_type=state_type,
            module=getattr(plugin_info, "module", None),
            class_name=getattr(plugin_info, "class_name", None),
            package_name=getattr(plugin_info, "package_name", None),
            file_name=getattr(plugin_info, "file_name", None),
        )
        for outcome_name in list(outcomes or getattr(plugin_info, "outcomes", []) or []):
            model.add_outcome(Outcome(name=outcome_name))
        return model

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
        if is_concurrence:
            model: StateMachine | Concurrence = Concurrence(
                name=name,
                description=description or "",
                default_outcome=default_outcome,
                remappings=dict(remappings or {}),
                parameter_mappings=dict(parameter_mappings or {}),
            )
        else:
            model = StateMachine(
                name=name,
                description=description or "",
                start_state=start_state,
                remappings=dict(remappings or {}),
                parameter_mappings=dict(parameter_mappings or {}),
            )
        for outcome_name in outcomes or []:
            model.add_outcome(Outcome(name=outcome_name))
        return model

    def _create_connection_view(self, from_node, to_node, outcome: str) -> ConnectionLine:
        connection = ConnectionLine(from_node, to_node, outcome)
        self.canvas.scene.addItem(connection)
        self.canvas.scene.addItem(connection.arrow_head)
        self.canvas.scene.addItem(connection.label_bg)
        self.canvas.scene.addItem(connection.label)
        self.connections.append(connection)
        for existing_connection in self.connections:
            existing_connection.update_position()
        return connection

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
        parent_model = (
            self.root_model if parent_container is None else parent_container.model
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
        self.canvas.clear_pending_placement()
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()
        self.text_blocks.clear()

    def sync_current_container_layout(self) -> None:
        container_model = self.current_container_model
        for state_name, state_view in self.state_nodes.items():
            container_model.layout.set_state_position(
                state_name,
                float(state_view.pos().x()),
                float(state_view.pos().y()),
            )
        for outcome_name, outcome_view in self.final_outcomes.items():
            container_model.layout.set_outcome_position(
                outcome_name,
                float(outcome_view.pos().x()),
                float(outcome_view.pos().y()),
            )
        for text_block_view in self.text_blocks:
            text_block_view.model.x = float(text_block_view.pos().x())
            text_block_view.model.y = float(text_block_view.pos().y())
            text_block_view.model.content = text_block_view.content

    def update_container_controls(self) -> None:
        if not hasattr(self, "root_sm_name_edit"):
            return
        model = self.current_container_model
        self.root_sm_name_edit.blockSignals(True)
        self.root_sm_description_edit.blockSignals(True)
        self.start_state_combo.blockSignals(True)

        if isinstance(model, StateMachine):
            self.root_sm_name_label.setText("<b>State Machine Name:</b>")
            self.start_state_label.setText("<b>Start State:</b>")
        else:
            self.root_sm_name_label.setText("<b>Concurrence Name:</b>")
            self.start_state_label.setText("<b>Default Outcome:</b>")

        self.root_sm_name_edit.setText(model.name)
        self.root_sm_description_edit.setText(model.description)

        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")
        if isinstance(model, StateMachine):
            for state_name in sorted(model.states.keys()):
                self.start_state_combo.addItem(state_name)
            current_value = model.start_state
        else:
            for outcome in model.outcomes:
                self.start_state_combo.addItem(outcome.name)
            current_value = model.default_outcome

        if current_value:
            index = self.start_state_combo.findText(current_value)
            self.start_state_combo.setCurrentIndex(index if index >= 0 else 0)
        else:
            self.start_state_combo.setCurrentIndex(0)

        self.root_sm_name_edit.blockSignals(False)
        self.root_sm_description_edit.blockSignals(False)
        self.start_state_combo.blockSignals(False)

    def on_root_sm_name_changed(self, text: str) -> None:
        model = self.current_container_model
        parent_model = self.current_parent_model
        text = text.strip()
        if parent_model is None:
            old_name = model.name
            model.name = text
            if isinstance(model, StateMachine):
                model.rename_transition_owner(old_name, text)
        else:
            old_name = model.name
            if text != old_name and text in parent_model.states:
                return
            parent_model.rename_state(old_name, text)
        self.refresh_breadcrumbs()

    def on_root_sm_description_changed(self, text: str) -> None:
        self.current_container_model.description = text

    def on_start_state_changed(self, text: str) -> None:
        if isinstance(self.current_container_model, StateMachine):
            self.start_state = None if text == "(None)" else text
        else:
            self.default_outcome = None if text == "(None)" else text

    def update_start_state_combo(self) -> None:
        self.update_container_controls()

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

    def find_target_view(self, target_name: str, source_container=None):
        target = self.state_nodes.get(target_name)
        if target is not None:
            return target
        return self.final_outcomes.get(target_name)

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
        return state_name in container_model.states or any(
            outcome.name == state_name for outcome in container_model.outcomes
        )

    def has_final_outcome_name_conflict(self, outcome_name: str) -> bool:
        return (
            outcome_name in self.final_outcomes
            or outcome_name in self.current_container_model.states
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

    def cancel_pending_node_placement(
        self,
        node: StateNode | ContainerStateNode | FinalOutcomeNode | TextBlockNode,
    ) -> None:
        if isinstance(node, FinalOutcomeNode):
            self.current_container_model.remove_outcome(node.name)
            self.final_outcomes.pop(node.name, None)
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

    def rename_final_outcome(
        self,
        outcome_node: FinalOutcomeNode,
        new_name: str,
    ) -> None:
        """Rename a final outcome and update all dependent model references."""

        old_name = outcome_node.name
        if old_name == new_name:
            return

        container_model = self.current_container_model
        container_model.rename_outcome(old_name, new_name)

        if old_name in self.final_outcomes:
            self.final_outcomes[new_name] = self.final_outcomes.pop(old_name)

        outcome_node.name = new_name
        outcome_node.update_attached_connections()

        parent_model = self.current_parent_model
        if parent_model is not None:
            if isinstance(parent_model, StateMachine):
                parent_model.rename_child_state_outcome(
                    container_model.name,
                    old_name,
                    new_name,
                )
            elif isinstance(parent_model, Concurrence):
                parent_model.rename_child_state_outcome(
                    container_model.name,
                    old_name,
                    new_name,
                )

        self.update_start_state_combo()
        self.refresh_connection_port_visibility()

    def register_connection_in_model(self, from_node, to_node, outcome: str) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.set_outcome_rule(to_node.name, from_node.name, outcome)
            return
        owner_model.add_transition(
            from_node.name,
            Transition(source_outcome=outcome, target=to_node.name),
        )

    def unregister_connection_in_model(self, connection: ConnectionLine) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.remove_outcome_rule(
                connection.to_node.name, connection.from_node.name
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
        for connection in list(outcome_node.connections):
            self.delete_connection_item(connection)

        self.final_outcomes.pop(outcome_node.name, None)
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

    def save_state_machine(self) -> None:
        self.sync_current_container_layout()
        validation = validate_model(self.root_model)
        errors = [f"- {item.message}" for item in validation.errors]

        if errors:
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
            if reply == QMessageBox.No:
                return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            if not file_path.lower().endswith(".xml"):
                file_path += ".xml"

            try:
                self.save_to_xml(file_path)
                self.current_file_path = file_path
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def get_free_position(self) -> QPointF:
        """Get a free position close to the current viewport center."""
        if not hasattr(self, "canvas"):
            return QPointF(100, 100)

        viewport_rect = self.canvas.viewport().rect()
        visible_rect = self.canvas.mapToScene(viewport_rect).boundingRect()
        center = visible_rect.center()

        occupied_positions = [
            item.pos()
            for item in list(self.state_nodes.values())
            + list(self.final_outcomes.values())
            + list(self.text_blocks)
        ]
        spacing_x = 180.0
        spacing_y = 130.0

        candidates = [QPointF(center.x(), center.y())]
        for radius in range(1, 6):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidates.append(
                        QPointF(center.x() + dx * spacing_x, center.y() + dy * spacing_y)
                    )

        def is_free(candidate: QPointF) -> bool:
            for occupied in occupied_positions:
                if (
                    abs(candidate.x() - occupied.x()) < spacing_x * 0.8
                    and abs(candidate.y() - occupied.y()) < spacing_y * 0.8
                ):
                    return False
            return True

        for candidate in candidates:
            if is_free(candidate):
                return candidate

        fallback_index = len(occupied_positions)
        return QPointF(
            center.x() + (fallback_index % 4) * spacing_x,
            center.y() + (fallback_index // 4) * spacing_y,
        )

    def load_from_xml(self, file_path: str) -> None:
        self.model_adapter.load_from_xml(file_path)
        self.current_file_path = file_path
        self.render_current_container(fit_view=True)
