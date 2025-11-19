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
    """Load a state machine from an XML file.

    Args:
        file_path: The path to the XML file to load.
    """

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
    """Reorganize all containers and their children after loading.

    Uses force-directed layout for graph positioning.
    """
    editor.reset_layout_rng()
    all_containers = []
    for node in editor.state_nodes.values():
        if isinstance(node, ContainerStateNode):
            all_containers.append(node)

    def get_nesting_depth(container):
        depth = 0
        current = container
        while hasattr(current, "parent_container") and current.parent_container:
            depth += 1
            current = current.parent_container
        return depth

    all_containers.sort(key=get_nesting_depth, reverse=True)

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
    """Layout children within a container using Fruchterman–Reingold algorithm.

    Uses physical simulation with Fruchterman-Reingold algorithm:
    - Repulsive forces between all nodes (prevent overlap)
    - Attractive forces along edges (keep connected nodes close)
    - Boundary forces (keep nodes within container)

    Args:
        container: The container state node to layout.
    """
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

    force_directed_layout(editor, container, all_nodes, graph)


def force_directed_layout(
    editor: "YasminEditor",
    container: ContainerStateNode,
    nodes: list,
    graph: dict,
) -> None:
    """Fruchterman-Reingold force-directed layout algorithm.

    Simulates physical forces:
    - Repulsion: All nodes repel each other (prevents overlap)
    - Attraction: Connected nodes attract each other (keeps graph connected)
    - Boundary: Nodes are pushed back if they leave the container

    Args:
        container: The container node.
        nodes: List of all nodes to position.
        graph: Adjacency dictionary mapping nodes to their neighbors.
    """
    import math

    rng = editor.layout_rng if getattr(editor, "layout_rng", None) is not None else random

    rect = container.rect()

    MARGIN_X = rect.width() * 0.05
    MARGIN_Y_TOP = rect.height() * 0.12
    MARGIN_Y_BOTTOM = rect.height() * 0.05

    area_width = rect.width() - (2 * MARGIN_X)
    area_height = rect.height() - MARGIN_Y_TOP - MARGIN_Y_BOTTOM

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
        current_pos = node.pos()
        if (
            rect.left() < current_pos.x() < rect.right()
            and rect.top() < current_pos.y() < rect.bottom()
        ):
            positions[node] = [current_pos.x(), current_pos.y()]
        else:
            x = rect.left() + MARGIN_X + rng.random() * area_width
            y = rect.top() + MARGIN_Y_TOP + rng.random() * area_height
            positions[node] = [x, y]

    area = area_width * area_height
    k = math.sqrt(area / len(nodes))

    ITERATIONS = 100
    initial_temp = area_width / 10
    temp = initial_temp

    for iteration in range(ITERATIONS):
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
                rect.left() + MARGIN_X,
                min(positions[node][0], rect.right() - MARGIN_X - node_w),
            )
            positions[node][1] = max(
                rect.top() + MARGIN_Y_TOP,
                min(positions[node][1], rect.bottom() - MARGIN_Y_BOTTOM - node_h),
            )

        temp = initial_temp * (1 - iteration / ITERATIONS)

    for node, (x, y) in positions.items():
        node.setPos(x, y)

    container.auto_resize_for_children()


def reposition_root_elements_after_resize(editor: "YasminEditor") -> None:
    """Reposition root-level elements using force-directed layout.

    Ensures optimal spacing and prevents overlapping.
    Includes final outcomes in the graph layout.
    """
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
    """Force-directed layout for root-level nodes.

    Uses Fruchterman-Reingold algorithm adapted for the root canvas.

    Args:
        nodes: List of all root-level nodes.
        graph: Adjacency dictionary.
    """
    import math

    rng = editor.layout_rng if getattr(editor, "layout_rng", None) is not None else random

    viewport_rect = editor.canvas.viewport().rect()
    viewport_width = viewport_rect.width()
    viewport_height = viewport_rect.height()

    MARGIN_X = viewport_width * 0.1
    MARGIN_Y = viewport_height * 0.1

    area_width = viewport_width - (2 * MARGIN_X)
    area_height = viewport_height - (2 * MARGIN_Y)

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
        current_pos = node.scenePos()
        if (
            MARGIN_X < current_pos.x() < viewport_width - MARGIN_X
            and MARGIN_Y < current_pos.y() < viewport_height - MARGIN_Y
        ):
            positions[node] = [current_pos.x(), current_pos.y()]
        else:
            x = MARGIN_X + rng.random() * area_width
            y = MARGIN_Y + rng.random() * area_height
            positions[node] = [x, y]

    area = area_width * area_height
    k = math.sqrt(area / len(nodes)) * 1.5

    ITERATIONS = 150
    initial_temp = area_width / 8
    temp = initial_temp

    for iteration in range(ITERATIONS):
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
                MARGIN_X, min(positions[node][0], viewport_width - MARGIN_X - node_w)
            )
            positions[node][1] = max(
                MARGIN_Y, min(positions[node][1], viewport_height - MARGIN_Y - node_h)
            )

        temp = initial_temp * (1 - iteration / ITERATIONS)

    for node, (x, y) in positions.items():
        node.setPos(x, y)

    for node in nodes:
        if hasattr(node, "connections"):
            for conn in node.connections:
                conn.update_position()


def load_states_from_xml(
    editor: "YasminEditor",
    parent_elem: ET.Element,
    parent_container: ContainerStateNode = None,
) -> None:
    """Recursively load states from XML, handling nested containers.

    Args:
        parent_elem: The parent XML element.
        parent_container: The parent container node (None for root).
    """
    for elem in parent_elem:
        if elem.tag == "State" or (elem.tag == "StateMachine" and elem.get("file_name")):
            state_name = elem.get("name")
            state_type = elem.get("type")
            remappings = load_remappings(elem)

            plugin_info = None
            if state_type == "py":
                module = elem.get("module")
                class_name = elem.get("class")
                for plugin in editor.plugin_manager.python_plugins:
                    if plugin.module == module and plugin.class_name == class_name:
                        plugin_info = plugin
                        break
            elif state_type == "cpp":
                class_name = elem.get("class")
                for plugin in editor.plugin_manager.cpp_plugins:
                    if plugin.class_name == class_name:
                        plugin_info = plugin
                        break
            elif state_type == "xml":
                file_name = elem.get("file_name")
                for plugin in editor.plugin_manager.xml_files:
                    if plugin.file_name == file_name:
                        plugin_info = plugin
                        break

            if plugin_info:
                node = StateNode(state_name, plugin_info, 0, 0, remappings)
                if parent_container is None:
                    editor.canvas.scene.addItem(node)
                    editor.state_nodes[state_name] = node
                else:
                    parent_container.add_child_state(node)
                    full_name = f"{parent_container.name}.{state_name}"
                    editor.state_nodes[full_name] = node

        elif elem.tag == "StateMachine" and not elem.get("file_name"):
            state_name = elem.get("name")
            outcomes_str = elem.get("outcomes", "")
            init_state = elem.get("start_state", "")
            remappings = load_remappings(elem)
            outcomes = outcomes_str.split() if outcomes_str else []

            node = ContainerStateNode(
                state_name, 0, 0, False, remappings, outcomes, init_state
            )
            if parent_container is None:
                editor.canvas.scene.addItem(node)
                editor.state_nodes[state_name] = node
            else:
                parent_container.add_child_state(node)
                full_name = f"{parent_container.name}.{state_name}"
                editor.state_nodes[full_name] = node

            for outcome in outcomes:
                outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                node.add_final_outcome(outcome_node)

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
            if parent_container is None:
                editor.canvas.scene.addItem(node)
                editor.state_nodes[state_name] = node
            else:
                parent_container.add_child_state(node)
                full_name = f"{parent_container.name}.{state_name}"
                editor.state_nodes[full_name] = node
            for outcome in outcomes:
                outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                node.add_final_outcome(outcome_node)

            load_states_from_xml(editor, elem, node)


def load_remappings(elem: ET.Element) -> dict:
    """Load remappings from an XML element.

    Args:
        elem: The XML element to load remappings from.

    Returns:
        dict: Dictionary of remappings (old -> new).
    """
    remappings = {}
    for remap in elem.findall("Remap"):
        from_key = remap.get("old", "")
        to_key = remap.get("new", "")
        if from_key and to_key:
            remappings[from_key] = to_key
    return remappings


def load_transitions_from_xml(
    editor: "YasminEditor",
    parent_elem: ET.Element,
    parent_container: ContainerStateNode = None,
) -> None:
    """Recursively load transitions from XML.

    Args:
        parent_elem: The parent XML element.
        parent_container: The parent container node (None for root).
    """
    for elem in parent_elem:
        if elem.tag in ["State", "StateMachine", "Concurrence"]:
            state_name = elem.get("name")

            # Find the from_node
            if parent_container is None:
                from_node = editor.state_nodes.get(state_name)
            else:
                full_name = f"{parent_container.name}.{state_name}"
                from_node = editor.state_nodes.get(full_name)

            if from_node:
                final_outcome_names = set()
                if elem.tag in ["StateMachine", "Concurrence"] and hasattr(
                    from_node, "final_outcomes"
                ):
                    final_outcome_names = set(from_node.final_outcomes.keys())

                for transition in elem.findall("Transition"):
                    outcome = transition.get("from")
                    to_name = transition.get("to")

                    is_from_final_outcome = outcome in final_outcome_names

                    if is_from_final_outcome:
                        from_outcome = from_node.final_outcomes[outcome]

                        to_node = None

                        if parent_container is None:
                            to_node = editor.state_nodes.get(to_name)
                            if not to_node:
                                to_node = editor.final_outcomes.get(to_name)
                        elif to_name in parent_container.final_outcomes:
                            to_node = parent_container.final_outcomes[to_name]
                        else:
                            full_to_name = f"{parent_container.name}.{to_name}"
                            to_node = editor.state_nodes.get(full_to_name)

                        if to_node:
                            connection = ConnectionLine(from_outcome, to_node, outcome)
                            editor.canvas.scene.addItem(connection)
                            editor.canvas.scene.addItem(connection.arrow_head)
                            editor.canvas.scene.addItem(connection.label_bg)
                            editor.canvas.scene.addItem(connection.label)
                            editor.connections.append(connection)
                    else:
                        to_node = None

                        if parent_container is None:
                            to_node = editor.state_nodes.get(to_name)
                            if not to_node:
                                to_node = editor.final_outcomes.get(to_name)
                        else:
                            if to_name in parent_container.final_outcomes:
                                to_node = parent_container.final_outcomes[to_name]
                            else:
                                full_to_name = f"{parent_container.name}.{to_name}"
                                to_node = editor.state_nodes.get(full_to_name)

                        if to_node:
                            connection = ConnectionLine(from_node, to_node, outcome)
                            editor.canvas.scene.addItem(connection)
                            editor.canvas.scene.addItem(connection.arrow_head)
                            editor.canvas.scene.addItem(connection.label_bg)
                            editor.canvas.scene.addItem(connection.label)
                            editor.connections.append(connection)

            if elem.tag in ["StateMachine", "Concurrence"]:
                if parent_container is None:
                    container = editor.state_nodes.get(state_name)
                else:
                    full_name = f"{parent_container.name}.{state_name}"
                    container = editor.state_nodes.get(full_name)

                if container:
                    load_transitions_from_xml(editor, elem, container)
