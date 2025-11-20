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
from PyQt5.QtWidgets import QApplication, QInputDialog

from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.connection_line import ConnectionLine

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class XmlManager:
    def __init__(self, editor: "YasminEditor") -> None:
        self.editor = editor

    def add_remappings(self, elem: ET.Element, remappings: dict) -> None:
        for old, new in remappings.items():
            if old and new:
                remap = ET.SubElement(elem, "Remap")
                remap.set("old", old)
                remap.set("new", new)

    def save_container(
        self, parent_elem: ET.Element, state_node: StateNode, tag: str
    ) -> None:
        cont_elem = ET.SubElement(parent_elem, tag)
        cont_elem.set("name", state_node.name)
        if hasattr(state_node, "start_state") and state_node.start_state:
            cont_elem.set("start_state", state_node.start_state)
        if hasattr(state_node, "default_outcome") and state_node.default_outcome:
            cont_elem.set("default_outcome", state_node.default_outcome)
        if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
            cont_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))
        if state_node.remappings:
            self.add_remappings(cont_elem, state_node.remappings)
        if hasattr(state_node, "child_states") and state_node.child_states:
            self.save_states_to_xml(cont_elem, state_node.child_states)
        self.save_transitions(cont_elem, state_node)
        if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
            for outcome_node in state_node.final_outcomes.values():
                self.save_transitions(cont_elem, outcome_node)

    def save_to_xml(self, file_path: str) -> None:
        """Save state machine to XML."""
        sm_name = (
            self.editor.root_sm_name
            or QInputDialog.getText(
                self.editor, "State Machine Name", "Enter state machine name (optional):"
            )[0]
        )
        root = ET.Element("StateMachine")
        root.set("name", sm_name)
        root.set("outcomes", " ".join(self.editor.final_outcomes.keys()))
        if self.editor.start_state:
            root.set("start_state", self.editor.start_state)
        root_level_states = {
            name: node
            for name, node in self.editor.state_nodes.items()
            if not hasattr(node, "parent_container") or node.parent_container is None
        }
        self.save_states_to_xml(root, root_level_states)
        tree = ET.ElementTree(root)
        tree.write(
            file_path if file_path.lower().endswith(".xml") else file_path + ".xml",
            encoding="utf-8",
            xml_declaration=True,
            pretty_print=True,
        )

    def save_states_to_xml(self, parent_elem: ET.Element, state_nodes_dict: dict) -> None:
        """Recursively save states to XML."""
        for state_node in state_nodes_dict.values():
            if state_node.is_concurrence:
                self.save_container(parent_elem, state_node, "Concurrence")
            elif state_node.is_state_machine:
                self.save_container(parent_elem, state_node, "StateMachine")
            else:
                tag = (
                    "State"
                    if state_node.plugin_info.plugin_type != "xml"
                    else "StateMachine"
                )
                state_elem = ET.SubElement(parent_elem, tag)
                state_elem.set("name", state_node.name)
                if state_node.plugin_info:
                    ptype = state_node.plugin_info.plugin_type
                    state_elem.set(
                        "type", {"python": "py", "cpp": "cpp", "xml": "xml"}[ptype]
                    )
                    if ptype == "python":
                        if state_node.plugin_info.module:
                            state_elem.set("module", state_node.plugin_info.module)
                        if state_node.plugin_info.class_name:
                            state_elem.set("class", state_node.plugin_info.class_name)
                    elif ptype == "cpp" and state_node.plugin_info.class_name:
                        state_elem.set("class", state_node.plugin_info.class_name)
                    elif ptype == "xml":
                        if state_node.plugin_info.file_name:
                            state_elem.set("file_name", state_node.plugin_info.file_name)
                            state_elem.set("package", state_node.plugin_info.package_name)
                if state_node.remappings:
                    self.add_remappings(state_elem, state_node.remappings)
                self.save_transitions(state_elem, state_node)

    def save_transitions(self, parent_elem: ET.Element, state_node: StateNode) -> None:
        """Save transitions for a state node."""
        for connection in state_node.connections:
            if connection.from_node == state_node:
                transition = ET.SubElement(parent_elem, "Transition")
                if connection.outcome:
                    transition.set("from", connection.outcome)
                if connection.to_node and connection.to_node.name:
                    transition.set("to", connection.to_node.name)

    def load_from_xml(self, file_path: str) -> None:
        """Load state machine from XML file."""
        tree = ET.parse(file_path)
        root = tree.getroot()

        sm_name = root.get("name", "")
        if sm_name:
            self.editor.root_sm_name = sm_name
            self.editor.root_sm_name_edit.setText(sm_name)

        start_state = root.get("start_state", "")

        if not start_state:
            for elem in root:
                if elem.tag == "State":
                    start_state = elem.get("name", "")
                    break

        if start_state:
            self.editor.start_state = start_state

        self.load_states_from_xml(root)

        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()
            for i, outcome in enumerate(outcomes):
                y = 200 + i * 120
                node = FinalOutcomeNode(outcome, 800, y)
                self.editor.canvas.scene.addItem(node)
                self.editor.final_outcomes[outcome] = node

        self.editor.update_start_state_combo()

        if start_state:
            index = self.editor.start_state_combo.findText(start_state)
            if index >= 0:
                self.editor.start_state_combo.setCurrentIndex(index)

        self.load_transitions_from_xml(root, None)
        self.reorganize_all_containers()
        QApplication.processEvents()

        self.reposition_root_elements_after_resize()

        for state in self.editor.state_nodes.values():
            if hasattr(state, "connections"):
                for conn in state.connections:
                    conn.update_position()

        for outcome in self.editor.final_outcomes.values():
            if hasattr(outcome, "connections"):
                for conn in outcome.connections:
                    conn.update_position()

        for _ in range(3):
            self.editor.canvas.scene.update()
            QApplication.processEvents()

    def reorganize_all_containers(self) -> None:
        """Reorganize containers and children after loading."""
        all_containers = [
            node
            for node in self.editor.state_nodes.values()
            if isinstance(node, ContainerStateNode)
        ]
        all_containers.sort(key=lambda c: c.name.count("."), reverse=True)

        for container in all_containers:
            self.layout_container_force_directed(container)

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
            self.editor.canvas.scene.update()
            QApplication.processEvents()

    def layout_container_force_directed(self, container: ContainerStateNode) -> None:
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
        self.force_directed_layout_generic(
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
        self,
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

        rng = (
            self.editor.layout_rng
            if getattr(self.editor, "layout_rng", None) is not None
            else random
        )

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

    def reposition_root_elements_after_resize(self) -> None:
        """Reposition root elements using force-directed layout."""
        root_nodes = []
        for node in self.editor.state_nodes.values():
            if not hasattr(node, "parent_container") or node.parent_container is None:
                root_nodes.append(node)

        root_final_outcomes = list(self.editor.final_outcomes.values())
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
                outcome.setPos(
                    x_pos + 600, 120 + root_final_outcomes.index(outcome) * 150
                )
            return

        self.force_directed_layout_root(all_nodes, graph)

    def force_directed_layout_root(
        self,
        nodes: list,
        graph: dict,
    ) -> None:
        """Force-directed layout for root-level nodes."""
        viewport_rect = self.editor.canvas.viewport().rect()
        margin_x = viewport_rect.width() * 0.1
        margin_y = viewport_rect.height() * 0.1
        self.force_directed_layout_generic(
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

    def add_node_to_editor_or_container(self, node, state_name, parent_container):

        if parent_container is None:
            self.editor.canvas.scene.addItem(node)
            self.editor.state_nodes[state_name] = node

        else:
            parent_container.add_child_state(node)
            self.editor.state_nodes[f"{parent_container.name}.{state_name}"] = node

    def load_states_from_xml(
        self,
        parent_elem: ET.Element,
        parent_container: ContainerStateNode = None,
    ) -> None:
        """Recursively load states from XML."""
        for elem in parent_elem:
            if elem.tag == "State" or (
                elem.tag == "StateMachine" and elem.get("file_name")
            ):
                state_name = elem.get("name")
                state_type = elem.get("type")
                remappings = self.load_remappings(elem)

                plugin_info = None
                if state_type == "py":
                    module = elem.get("module")
                    class_name = elem.get("class")
                    plugin_info = next(
                        (
                            p
                            for p in self.editor.plugin_manager.python_plugins
                            if p.module == module and p.class_name == class_name
                        ),
                        None,
                    )
                elif state_type == "cpp":
                    class_name = elem.get("class")
                    plugin_info = next(
                        (
                            p
                            for p in self.editor.plugin_manager.cpp_plugins
                            if p.class_name == class_name
                        ),
                        None,
                    )
                elif state_type == "xml":
                    file_name = elem.get("file_name")
                    plugin_info = next(
                        (
                            p
                            for p in self.editor.plugin_manager.xml_files
                            if p.file_name == file_name
                        ),
                        None,
                    )

                if plugin_info:
                    node = StateNode(state_name, plugin_info, 0, 0, remappings)
                    self.add_node_to_editor_or_container(
                        node, state_name, parent_container
                    )

            elif elem.tag == "StateMachine" and not elem.get("file_name"):
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                init_state = elem.get("start_state", "")
                remappings = self.load_remappings(elem)
                outcomes = outcomes_str.split() if outcomes_str else []

                node = ContainerStateNode(
                    state_name, 0, 0, False, remappings, outcomes, init_state
                )
                self.add_node_to_editor_or_container(node, state_name, parent_container)

                for outcome in outcomes:
                    node.add_final_outcome(
                        FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    )

                self.load_states_from_xml(elem, node)

            elif elem.tag == "Concurrence":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                default_outcome = elem.get("default_outcome", None)
                remappings = self.load_remappings(elem)
                outcomes = outcomes_str.split() if outcomes_str else []

                node = ContainerStateNode(
                    state_name, 0, 0, True, remappings, outcomes, None, default_outcome
                )
                self.add_node_to_editor_or_container(node, state_name, parent_container)

                for outcome in outcomes:
                    node.add_final_outcome(
                        FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    )

                self.load_states_from_xml(elem, node)

    def load_remappings(self, elem: ET.Element) -> dict:
        """Load remappings from XML element."""
        remappings = {}
        for remap in elem.findall("Remap"):
            from_key = remap.get("old", "")
            to_key = remap.get("new", "")
            if from_key and to_key:
                remappings[from_key] = to_key
        return remappings

    def find_to_node(
        self, to_name: str, parent_container: ContainerStateNode = None
    ) -> StateNode:

        if parent_container is None:
            return self.editor.state_nodes.get(to_name) or self.editor.final_outcomes.get(
                to_name
            )

        return parent_container.final_outcomes.get(
            to_name
        ) or self.editor.state_nodes.get(f"{parent_container.name}.{to_name}")

    def add_connection(
        self, from_node: StateNode, to_node: StateNode, outcome: str
    ) -> None:
        connection = ConnectionLine(from_node, to_node, outcome)
        self.editor.canvas.scene.addItem(connection)
        self.editor.canvas.scene.addItem(connection.arrow_head)
        self.editor.canvas.scene.addItem(connection.label_bg)
        self.editor.canvas.scene.addItem(connection.label)
        self.editor.connections.append(connection)

    def load_transitions_from_xml(
        self,
        parent_elem: ET.Element,
        parent_container: ContainerStateNode = None,
    ) -> None:
        """Recursively load transitions from XML."""
        for elem in parent_elem:
            if elem.tag in ["State", "StateMachine", "Concurrence"]:
                state_name = elem.get("name")

                from_node = self.editor.state_nodes.get(
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
                        to_node = self.find_to_node(to_name, parent_container)

                        if to_node:
                            self.add_connection(from_node_actual, to_node, outcome)

                if elem.tag in ["StateMachine", "Concurrence"]:
                    container = self.editor.state_nodes.get(
                        state_name
                        if parent_container is None
                        else f"{parent_container.name}.{state_name}"
                    )
                    if container:
                        self.load_transitions_from_xml(elem, container)
