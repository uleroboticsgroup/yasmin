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

from typing import TYPE_CHECKING, Optional

from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def create_empty_model() -> StateMachine:
    return StateMachine(name="")


def sync_model_from_editor(editor: "YasminEditor") -> StateMachine:
    model = StateMachine(
        name=editor.root_sm_name,
        description=editor.root_sm_description_edit.text().strip(),
        start_state=editor.start_state,
    )

    for key_data in list(getattr(editor, "_blackboard_keys", [])):
        model.add_key(
            Key(
                name=str(key_data.get("name", "") or "").strip(),
                key_type=str(key_data.get("key_type", "in") or "in").strip(),
                description=str(key_data.get("description", "") or "").strip(),
                default_type=str(key_data.get("default_type", "") or "").strip(),
                default_value=key_data.get("default_value"),
            )
        )

    root_state_nodes = [
        node
        for node in editor.state_nodes.values()
        if getattr(node, "parent_container", None) is None
    ]
    for node in sorted(root_state_nodes, key=lambda item: item.name.lower()):
        state = _build_state_from_node(node)
        model.add_state(state)
        model.layout.set_state_position(node.name, node.pos().x(), node.pos().y())
        model.remappings[state.name] = dict(getattr(node, "remappings", {}) or {})
        _append_state_machine_connections(model, node, None)

    for outcome_node in sorted(editor.final_outcomes.values(), key=lambda item: item.name.lower()):
        model.add_outcome(Outcome(name=outcome_node.name, description=outcome_node.description))
        model.layout.set_outcome_position(
            outcome_node.name, outcome_node.pos().x(), outcome_node.pos().y()
        )

    return model


def load_model_into_editor(editor: "YasminEditor", model: StateMachine) -> None:
    editor.root_sm_name = model.name
    editor.start_state = model.start_state
    editor.root_sm_name_edit.setText(model.name)
    editor.root_sm_description_edit.setText(model.description)
    editor.set_blackboard_keys(
        [
            {
                "name": key.name,
                "key_type": key.key_type,
                "description": key.description,
                "default_type": key.default_type,
                "default_value": "" if key.default_value is None else str(key.default_value),
            }
            for key in model.keys
        ],
        sync=False,
    )

    for state in model.states.values():
        _create_gui_state(editor, state, None, model)

    for outcome in model.outcomes:
        position = model.layout.get_outcome_position(outcome.name)
        x = position.x if position is not None else 600.0
        y = position.y if position is not None else float(len(editor.final_outcomes) * 150)
        node = FinalOutcomeNode(outcome.name, x, y, description=outcome.description)
        editor.canvas.scene.addItem(node)
        editor.final_outcomes[outcome.name] = node

    _create_state_machine_connections(editor, model, None)

    editor.update_start_state_combo()
    if model.start_state:
        index = editor.start_state_combo.findText(model.start_state)
        if index >= 0:
            editor.start_state_combo.setCurrentIndex(index)

    editor.sync_blackboard_keys()


def _build_state_from_node(node: StateNode | ContainerStateNode) -> State:
    if isinstance(node, ContainerStateNode):
        if node.is_concurrence:
            model = Concurrence(
                name=node.name,
                description=getattr(node, "description", "") or "",
                default_outcome=getattr(node, "default_outcome", None),
            )

            for child in sorted(node.child_states.values(), key=lambda item: item.name.lower()):
                child_state = _build_state_from_node(child)
                model.add_state(child_state)
                model.layout.set_state_position(child.name, child.pos().x(), child.pos().y())

            for outcome_node in sorted(node.final_outcomes.values(), key=lambda item: item.name.lower()):
                model.add_outcome(
                    Outcome(name=outcome_node.name, description=outcome_node.description)
                )
                model.layout.set_outcome_position(
                    outcome_node.name, outcome_node.pos().x(), outcome_node.pos().y()
                )

            for child in node.child_states.values():
                for connection in getattr(child, "connections", []):
                    if connection.from_node != child:
                        continue
                    target = connection.to_node
                    if isinstance(target, FinalOutcomeNode) and target.parent_container == node:
                        model.set_outcome_rule(target.name, child.name, connection.outcome)

            return model

        model = StateMachine(
            name=node.name,
            description=getattr(node, "description", "") or "",
            start_state=getattr(node, "start_state", None),
        )

        for child in sorted(node.child_states.values(), key=lambda item: item.name.lower()):
            child_state = _build_state_from_node(child)
            model.add_state(child_state)
            model.layout.set_state_position(child.name, child.pos().x(), child.pos().y())
            model.remappings[child.name] = dict(getattr(child, "remappings", {}) or {})
            _append_state_machine_connections(model, child, node)

        for outcome_node in sorted(node.final_outcomes.values(), key=lambda item: item.name.lower()):
            model.add_outcome(Outcome(name=outcome_node.name, description=outcome_node.description))
            model.layout.set_outcome_position(
                outcome_node.name, outcome_node.pos().x(), outcome_node.pos().y()
            )
            for connection in getattr(outcome_node, "connections", []):
                if connection.from_node != outcome_node:
                    continue
                model.add_transition(
                    outcome_node.name,
                    Transition(
                        source_outcome=connection.outcome,
                        target=_get_target_name(connection.to_node),
                    ),
                )

        return model

    plugin_info = node.plugin_info
    state_type = None
    module = None
    class_name = None
    package_name = None
    file_name = None

    if plugin_info is not None:
        plugin_type = getattr(plugin_info, "plugin_type", None)
        state_type = {"python": "py", "cpp": "cpp", "xml": "xml"}.get(
            plugin_type, plugin_type
        )
        module = getattr(plugin_info, "module", None)
        class_name = getattr(plugin_info, "class_name", None)
        package_name = getattr(plugin_info, "package_name", None)
        file_name = getattr(plugin_info, "file_name", None)

    state = State(
        name=node.name,
        description=getattr(node, "description", "") or "",
        state_type=state_type,
        module=module,
        class_name=class_name,
        package_name=package_name,
        file_name=file_name,
    )

    for outcome_name in list(getattr(plugin_info, "outcomes", []) or []):
        state.add_outcome(Outcome(name=outcome_name))

    return state


def _append_state_machine_connections(
    model: StateMachine,
    node: StateNode | ContainerStateNode,
    parent_container: Optional[ContainerStateNode],
) -> None:
    for connection in getattr(node, "connections", []):
        if connection.from_node != node:
            continue
        model.add_transition(
            node.name,
            Transition(
                source_outcome=connection.outcome,
                target=_get_target_name(connection.to_node),
            ),
        )


def _get_target_name(node: StateNode | ContainerStateNode | FinalOutcomeNode) -> str:
    return node.name


def _create_gui_state(
    editor: "YasminEditor",
    state: State,
    parent_container: Optional[ContainerStateNode],
    parent_model: StateMachine | Concurrence,
) -> StateNode | ContainerStateNode:
    position = parent_model.layout.get_state_position(state.name)
    x = position.x if position is not None else 0.0
    y = position.y if position is not None else 0.0

    if isinstance(state, StateMachine):
        node = ContainerStateNode(
            state.name,
            x,
            y,
            False,
            dict(parent_model.remappings.get(state.name, {}))
            if isinstance(parent_model, StateMachine)
            else {},
            [outcome.name for outcome in state.outcomes],
            state.start_state,
            None,
            description=state.description,
        )
    elif isinstance(state, Concurrence):
        node = ContainerStateNode(
            state.name,
            x,
            y,
            True,
            dict(parent_model.remappings.get(state.name, {}))
            if isinstance(parent_model, StateMachine)
            else {},
            [outcome.name for outcome in state.outcomes],
            None,
            state.default_outcome,
            description=state.description,
        )
    else:
        plugin_info = _find_plugin_info(editor, state)
        if plugin_info is None:
            raise ValueError(f"Could not resolve plugin for state '{state.name}'")
        node = StateNode(
            state.name,
            plugin_info,
            x,
            y,
            dict(parent_model.remappings.get(state.name, {}))
            if isinstance(parent_model, StateMachine)
            else {},
            state.description,
        )

    if parent_container is None:
        editor.canvas.scene.addItem(node)
        editor.state_nodes[state.name] = node
        full_prefix = state.name
    else:
        parent_container.add_child_state(node)
        editor.state_nodes[f"{parent_container.name}.{state.name}"] = node
        full_prefix = f"{parent_container.name}.{state.name}"

    if isinstance(state, StateMachine):
        for child in state.states.values():
            _create_gui_state(editor, child, node, state)

        for outcome in state.outcomes:
            outcome_position = state.layout.get_outcome_position(outcome.name)
            ox = outcome_position.x if outcome_position is not None else 0.0
            oy = outcome_position.y if outcome_position is not None else 0.0
            node.add_final_outcome(
                FinalOutcomeNode(
                    outcome.name,
                    ox,
                    oy,
                    inside_container=True,
                    description=outcome.description,
                )
            )

        _create_state_machine_connections(editor, state, node)
        node.auto_resize_for_children()
    elif isinstance(state, Concurrence):
        for child in state.states.values():
            _create_gui_state(editor, child, node, state)

        for outcome in state.outcomes:
            outcome_position = state.layout.get_outcome_position(outcome.name)
            ox = outcome_position.x if outcome_position is not None else 0.0
            oy = outcome_position.y if outcome_position is not None else 0.0
            node.add_final_outcome(
                FinalOutcomeNode(
                    outcome.name,
                    ox,
                    oy,
                    inside_container=True,
                    description=outcome.description,
                )
            )

        _create_concurrence_connections(editor, state, node)
        node.auto_resize_for_children()

    return node


def _create_state_machine_connections(
    editor: "YasminEditor",
    model: StateMachine,
    container_node: Optional[ContainerStateNode],
) -> None:
    for owner_name, transitions in model.transitions.items():
        for transition in transitions:
            from_node = _resolve_state_machine_owner(editor, model, container_node, owner_name)
            to_node = _resolve_target_node(editor, container_node, transition.target)
            if from_node is None or to_node is None:
                continue
            _add_connection(editor, from_node, to_node, transition.source_outcome)


def _create_concurrence_connections(
    editor: "YasminEditor",
    model: Concurrence,
    container_node: ContainerStateNode,
) -> None:
    for outcome_name, requirements in model.outcome_map.items():
        outcome_node = container_node.final_outcomes.get(outcome_name)
        if outcome_node is None:
            continue
        for state_name, state_outcome in requirements.items():
            from_node = container_node.child_states.get(state_name)
            if from_node is None:
                continue
            _add_connection(editor, from_node, outcome_node, state_outcome)


def _resolve_state_machine_owner(
    editor: "YasminEditor",
    model: StateMachine,
    container_node: Optional[ContainerStateNode],
    owner_name: str,
):
    if container_node is None:
        return editor.state_nodes.get(owner_name) or editor.final_outcomes.get(owner_name)
    return container_node.child_states.get(owner_name) or container_node.final_outcomes.get(owner_name)


def _resolve_target_node(
    editor: "YasminEditor",
    container_node: Optional[ContainerStateNode],
    target_name: str,
):
    if container_node is None:
        return editor.state_nodes.get(target_name) or editor.final_outcomes.get(target_name)

    current_container = container_node
    while current_container is not None:
        local_target = current_container.child_states.get(target_name)
        if local_target is not None:
            return local_target

        local_outcome = current_container.final_outcomes.get(target_name)
        if local_outcome is not None:
            return local_outcome

        current_container = current_container.parent_container

    return editor.state_nodes.get(target_name) or editor.final_outcomes.get(target_name)


def _add_connection(editor: "YasminEditor", from_node, to_node, outcome: str) -> None:
    connection = ConnectionLine(from_node, to_node, outcome)
    editor.canvas.scene.addItem(connection)
    editor.canvas.scene.addItem(connection.arrow_head)
    editor.canvas.scene.addItem(connection.label_bg)
    editor.canvas.scene.addItem(connection.label)
    editor.connections.append(connection)


def _find_plugin_info(editor: "YasminEditor", state: State):
    if state.state_type == "py":
        return next(
            (
                plugin
                for plugin in editor.plugin_manager.python_plugins
                if plugin.module == state.module and plugin.class_name == state.class_name
            ),
            None,
        )

    if state.state_type == "cpp":
        return next(
            (
                plugin
                for plugin in editor.plugin_manager.cpp_plugins
                if plugin.class_name == state.class_name
            ),
            None,
        )

    if state.state_type == "xml":
        plugin = next(
            (
                item
                for item in editor.plugin_manager.xml_files
                if item.file_name == state.file_name
                and (state.package_name is None or item.package_name == state.package_name)
            ),
            None,
        )
        if plugin is not None:
            return plugin
        return next(
            (item for item in editor.plugin_manager.xml_files if item.file_name == state.file_name),
            None,
        )

    return None
