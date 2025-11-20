# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from random import random
from lxml import etree as ET
from typing import TYPE_CHECKING
from PyQt5.QtWidgets import QApplication

from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.connection_line import ConnectionLine

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def load_from_xml(editor: "YasminEditor", file_path: str) -> None:
    """Load state machine from XML file."""
    tree = ET.parse(file_path)
    root = tree.getroot()

    sm_name = root.get("name", "")
    if sm_name:
        editor.root_sm_name = sm_name
        editor.root_sm_name_edit.setText(sm_name)

    start_state = root.get("start_state", "")

    if not start_state:
        for elem in root:
            if elem.tag == "State":
                start_state = elem.get("name", "")
                break

    if start_state:
        editor.start_state = start_state

    load_states_from_xml(editor, root)

    outcomes_str = root.get("outcomes", "")
    if outcomes_str:
        outcomes = outcomes_str.split()
        for i, outcome in enumerate(outcomes):
            y = 200 + i * 120
            node = FinalOutcomeNode(outcome, 800, y)
            editor.canvas.scene.addItem(node)
            editor.final_outcomes[outcome] = node

    editor.update_start_state_combo()

    if start_state:
        index = editor.start_state_combo.findText(start_state)
        if index >= 0:
            editor.start_state_combo.setCurrentIndex(index)

    load_transitions_from_xml(editor, root, None)
    reorganize_all_containers(editor)
    QApplication.processEvents()

    reposition_root_elements_after_resize(editor)

    for state in editor.state_nodes.values():
        if hasattr(state, "connections"):
            for conn in state.connections:
                conn.update_position()

    for outcome in editor.final_outcomes.values():
        if hasattr(outcome, "connections"):
            for conn in outcome.connections:
                conn.update_position()

    for _ in range(3):
        editor.canvas.scene.update()
        QApplication.processEvents()


def reorganize_all_containers(editor: "YasminEditor") -> None:
    """Reorganize containers and children after loading."""
    editor.reset_layout_rng()
    all_containers = [
        node
        for node in editor.state_nodes.values()
        if isinstance(node, ContainerStateNode)
    ]
    all_containers.sort(key=lambda c: c.name.count("."), reverse=True)

    for container in all_containers:
        layout_container_force_directed(editor, container)

    for container in all_containers:
        container.prepareGeometryChange()
        container.auto_resize_for_children()

    for container in all_containers:
        container.prepareGeometryChange()
        container.update()

        for child_state in container.child_states.values():
            child_state.prepareGeometryChange()
            child_state.update()
            if hasattr(child_state, "connections"):
                for conn in child_state.connections:
                    conn.update_position()

        for final_outcome in container.final_outcomes.values():
            final_outcome.prepareGeometryChange()
            final_outcome.update()
            if hasattr(final_outcome, "connections"):
                for conn in final_outcome.connections:
                    conn.update_position()

    for _ in range(4):
        editor.canvas.scene.update()
        QApplication.processEvents()


def layout_container_force_directed(
    editor: "YasminEditor", container: ContainerStateNode
) -> None:
    """Layout children in container using force-directed algorithm."""
    if not container.child_states and not container.final_outcomes:
        return

    nodes = list(container.child_states.values())
    final_outcomes = list(container.final_outcomes.values())

    if not nodes:
        rect = container.rect()
        y_start = rect.top() + 140
        for i, outcome in enumerate(final_outcomes):
            outcome.setPos(rect.left() + 120, y_start + i * 120)
        container.auto_resize_for_children()
        return

    all_nodes = nodes + final_outcomes
    graph = {node: [] for node in all_nodes}

    for node in nodes:
        if hasattr(node, "connections"):
            for conn in node.connections:
                if conn.from_node == node:
                    if conn.to_node in all_nodes:
                        graph[node].append(conn.to_node)

    rect = container.rect()
    force_directed_layout_generic(
        editor,
        all_nodes,
        graph,
        rect.left(),
        rect.top(),
        rect.right(),
        rect.bottom(),
        rect.width() * 0.05,
        rect.height() * 0.12,
        rect.height() * 0.05,
        0.0,
        0.0,
        0.0,
        1.2,
        1000,
        15,
        lambda n: n.pos(),
        lambda: container.auto_resize_for_children(),
    )


def force_directed_layout_generic(
    editor: "YasminEditor",
    nodes: list,
    graph: dict,
    rect_left: float,
    rect_top: float,
    rect_right: float,
    rect_bottom: float,
    margin_x: float,
    margin_y_top: float,
    margin_y_bottom: float,
    check_margin_x: float,
    check_margin_y_top: float,
    check_margin_y_bottom: float,
    k_multiplier: float,
    iterations: int,
    temp_divisor: float,
    get_pos_func,
    auto_resize_func=None,
) -> None:
    """Generic Fruchterman-Reingold force-directed layout algorithm."""
    import math

    rng = editor.layout_rng if getattr(editor, "layout_rng", None) is not None else random

    area_width = rect_right - rect_left - 2 * margin_x
    area_height = rect_bottom - rect_top - margin_y_top - margin_y_bottom

    node_dimensions = {}
    for node in nodes:
        if isinstance(node, ContainerStateNode):
            node.prepareGeometryChange()
            w, h = node.rect().width(), node.rect().height()
        else:
            w, h = node.boundingRect().width(), node.boundingRect().height()
        node_dimensions[node] = (w, h)

    positions = {}
    for node in nodes:
        current_pos = get_pos_func(node)
        if (
            rect_left + check_margin_x < current_pos.x() < rect_right - check_margin_x
            and rect_top + check_margin_y_top
            < current_pos.y()
            < rect_bottom - check_margin_y_bottom
        ):
            positions[node] = [current_pos.x(), current_pos.y()]
        else:
            x = rect_left + margin_x + rng.random() * area_width
            y = rect_top + margin_y_top + rng.random() * area_height
            positions[node] = [x, y]

    area = area_width * area_height
    k = math.sqrt(area / len(nodes)) * k_multiplier

    initial_temp = area_width / temp_divisor
    temp = initial_temp

    for iteration in range(iterations):
        displacement = {node: [0.0, 0.0] for node in nodes}

        for i, node1 in enumerate(nodes):
            for node2 in nodes[i + 1 :]:
                delta_x = positions[node1][0] - positions[node2][0]
                delta_y = positions[node1][1] - positions[node2][1]

                distance = math.sqrt(delta_x**2 + delta_y**2)
                if distance < 0.01:
                    distance = 0.01

                force = (k * k) / distance

                displacement[node1][0] += (delta_x / distance) * force
                displacement[node1][1] += (delta_y / distance) * force
                displacement[node2][0] -= (delta_x / distance) * force
                displacement[node2][1] -= (delta_y / distance) * force

        for node in nodes:
            for neighbor in graph.get(node, []):
                if neighbor not in nodes:
                    continue

                delta_x = positions[node][0] - positions[neighbor][0]
                delta_y = positions[node][1] - positions[neighbor][1]

                distance = math.sqrt(delta_x**2 + delta_y**2)
                if distance < 0.01:
                    distance = 0.01

                force = (distance * distance) / k

                displacement[node][0] -= (delta_x / distance) * force
                displacement[node][1] -= (delta_y / distance) * force
                displacement[neighbor][0] += (delta_x / distance) * force
                displacement[neighbor][1] += (delta_y / distance) * force

        for node in nodes:
            disp_x = displacement[node][0]
            disp_y = displacement[node][1]

            disp_length = math.sqrt(disp_x**2 + disp_y**2)
            if disp_length < 0.01:
                disp_length = 0.01

            limited_disp = min(disp_length, temp) / disp_length

            positions[node][0] += disp_x * limited_disp
            positions[node][1] += disp_y * limited_disp

            node_w, node_h = node_dimensions[node]
            positions[node][0] = max(
                rect_left + margin_x,
                min(positions[node][0], rect_right - margin_x - node_w),
            )
            positions[node][1] = max(
                rect_top + margin_y_top,
                min(positions[node][1], rect_bottom - margin_y_bottom - node_h),
            )

        temp = initial_temp * (1 - iteration / iterations)

    for node, (x, y) in positions.items():
        node.setPos(x, y)

    if auto_resize_func:
        auto_resize_func()


def reposition_root_elements_after_resize(editor: "YasminEditor") -> None:
    """Reposition root elements using force-directed layout."""
    root_nodes = []
    for node in editor.state_nodes.values():
        if not hasattr(node, "parent_container") or node.parent_container is None:
            root_nodes.append(node)

    root_final_outcomes = list(editor.final_outcomes.values())
    if not root_nodes and not root_final_outcomes:
        return

    all_nodes = root_nodes + root_final_outcomes
    graph = {node: [] for node in all_nodes}
    reverse_graph = {node: [] for node in all_nodes}

    for node in root_nodes:
        if hasattr(node, "connections"):
            for conn in node.connections:
                if conn.from_node == node:
                    if conn.to_node in all_nodes:
                        graph[node].append(conn.to_node)
                        reverse_graph[conn.to_node].append(node)

    total_edges = sum(len(neighbors) for neighbors in graph.values())
    if total_edges == 0 and len(all_nodes) > 1:
        x_pos = 120
        y_pos = 120
        for node in root_nodes:
            node.setPos(x_pos, y_pos)
            y_pos += 200
        for outcome in root_final_outcomes:
            outcome.setPos(x_pos + 600, 120 + root_final_outcomes.index(outcome) * 150)
        return

    force_directed_layout_root(editor, all_nodes, graph)


def force_directed_layout_root(
    editor: "YasminEditor",
    nodes: list,
    graph: dict,
) -> None:
    """Force-directed layout for root-level nodes."""
    viewport_rect = editor.canvas.viewport().rect()
    margin_x = viewport_rect.width() * 0.1
    margin_y = viewport_rect.height() * 0.1
    force_directed_layout_generic(
        editor,
        nodes,
        graph,
        viewport_rect.left(),
        viewport_rect.top(),
        viewport_rect.right(),
        viewport_rect.bottom(),
        margin_x,
        margin_y,
        margin_y,
        margin_x,
        margin_y,
        margin_y,
        1.5,
        1000,
        15,
        lambda n: n.scenePos(),
        None,
    )


def add_node_to_editor_or_container(editor, node, state_name, parent_container):

    if parent_container is None:
        editor.canvas.scene.addItem(node)
        editor.state_nodes[state_name] = node

    else:
        parent_container.add_child_state(node)
        editor.state_nodes[f"{parent_container.name}.{state_name}"] = node


def load_states_from_xml(
    editor: "YasminEditor",
    parent_elem: ET.Element,
    parent_container: ContainerStateNode = None,
) -> None:
    """Recursively load states from XML."""
    for elem in parent_elem:
        if elem.tag == "State" or (elem.tag == "StateMachine" and elem.get("file_name")):
            state_name = elem.get("name")
            state_type = elem.get("type")
            remappings = load_remappings(elem)

            plugin_info = None
            if state_type == "py":
                module = elem.get("module")
                class_name = elem.get("class")
                plugin_info = next(
                    (
                        p
                        for p in editor.plugin_manager.python_plugins
                        if p.module == module and p.class_name == class_name
                    ),
                    None,
                )
            elif state_type == "cpp":
                class_name = elem.get("class")
                plugin_info = next(
                    (
                        p
                        for p in editor.plugin_manager.cpp_plugins
                        if p.class_name == class_name
                    ),
                    None,
                )
            elif state_type == "xml":
                file_name = elem.get("file_name")
                plugin_info = next(
                    (
                        p
                        for p in editor.plugin_manager.xml_files
                        if p.file_name == file_name
                    ),
                    None,
                )

            if plugin_info:
                node = StateNode(state_name, plugin_info, 0, 0, remappings)
                add_node_to_editor_or_container(
                    editor, node, state_name, parent_container
                )

        elif elem.tag == "StateMachine" and not elem.get("file_name"):
            state_name = elem.get("name")
            outcomes_str = elem.get("outcomes", "")
            init_state = elem.get("start_state", "")
            remappings = load_remappings(elem)
            outcomes = outcomes_str.split() if outcomes_str else []

            node = ContainerStateNode(
                state_name, 0, 0, False, remappings, outcomes, init_state
            )
            add_node_to_editor_or_container(editor, node, state_name, parent_container)

            for outcome in outcomes:
                node.add_final_outcome(
                    FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                )

            load_states_from_xml(editor, elem, node)

        elif elem.tag == "Concurrence":
            state_name = elem.get("name")
            outcomes_str = elem.get("outcomes", "")
            default_outcome = elem.get("default_outcome", None)
            remappings = load_remappings(elem)
            outcomes = outcomes_str.split() if outcomes_str else []

            node = ContainerStateNode(
                state_name, 0, 0, True, remappings, outcomes, None, default_outcome
            )
            add_node_to_editor_or_container(editor, node, state_name, parent_container)

            for outcome in outcomes:
                node.add_final_outcome(
                    FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                )

            load_states_from_xml(editor, elem, node)


def load_remappings(elem: ET.Element) -> dict:
    """Load remappings from XML element."""
    remappings = {}
    for remap in elem.findall("Remap"):
        from_key = remap.get("old", "")
        to_key = remap.get("new", "")
        if from_key and to_key:
            remappings[from_key] = to_key
    return remappings


def find_to_node(
    editor: "YasminEditor", to_name: str, parent_container: ContainerStateNode = None
) -> "StateNode":

    if parent_container is None:
        return editor.state_nodes.get(to_name) or editor.final_outcomes.get(to_name)

    return parent_container.final_outcomes.get(to_name) or editor.state_nodes.get(
        f"{parent_container.name}.{to_name}"
    )


def add_connection(
    editor: "YasminEditor", from_node: "StateNode", to_node: "StateNode", outcome: str
) -> None:
    connection = ConnectionLine(from_node, to_node, outcome)
    editor.canvas.scene.addItem(connection)
    editor.canvas.scene.addItem(connection.arrow_head)
    editor.canvas.scene.addItem(connection.label_bg)
    editor.canvas.scene.addItem(connection.label)
    editor.connections.append(connection)


def load_transitions_from_xml(
    editor: "YasminEditor",
    parent_elem: ET.Element,
    parent_container: ContainerStateNode = None,
) -> None:
    """Recursively load transitions from XML."""
    for elem in parent_elem:
        if elem.tag in ["State", "StateMachine", "Concurrence"]:
            state_name = elem.get("name")

            from_node = editor.state_nodes.get(
                state_name
                if parent_container is None
                else f"{parent_container.name}.{state_name}"
            )

            if from_node:
                final_outcome_names = (
                    set(from_node.final_outcomes.keys())
                    if hasattr(from_node, "final_outcomes")
                    else set()
                )

                for transition in elem.findall("Transition"):
                    outcome = transition.get("from")
                    to_name = transition.get("to")

                    from_node_actual = (
                        from_node.final_outcomes[outcome]
                        if outcome in final_outcome_names
                        else from_node
                    )
                    to_node = find_to_node(editor, to_name, parent_container)

                    if to_node:
                        add_connection(editor, from_node_actual, to_node, outcome)

            if elem.tag in ["StateMachine", "Concurrence"]:
                container = editor.state_nodes.get(
                    state_name
                    if parent_container is None
                    else f"{parent_container.name}.{state_name}"
                )
                if container:
                    load_transitions_from_xml(editor, elem, container)
