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

from lxml import etree as ET
from typing import TYPE_CHECKING
from PyQt5.QtWidgets import QInputDialog
from yasmin_editor.editor_gui.state_node import StateNode

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def add_remappings(elem: ET.Element, remappings: dict) -> None:
    for old, new in remappings.items():
        if old and new:
            remap = ET.SubElement(elem, "Remap")
            remap.set("old", old)
            remap.set("new", new)


def save_container(parent_elem: ET.Element, state_node: StateNode, tag: str) -> None:
    cont_elem = ET.SubElement(parent_elem, tag)
    cont_elem.set("name", state_node.name)
    if hasattr(state_node, "start_state") and state_node.start_state:
        cont_elem.set("start_state", state_node.start_state)
    if hasattr(state_node, "default_outcome") and state_node.default_outcome:
        cont_elem.set("default_outcome", state_node.default_outcome)
    if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
        cont_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))
    if state_node.remappings:
        add_remappings(cont_elem, state_node.remappings)
    if hasattr(state_node, "child_states") and state_node.child_states:
        save_states_to_xml(cont_elem, state_node.child_states)
    save_transitions(cont_elem, state_node)
    if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
        for outcome_node in state_node.final_outcomes.values():
            save_transitions(cont_elem, outcome_node)


def save_to_xml(editor: "YasminEditor", file_path: str) -> None:
    """Save state machine to XML."""
    sm_name = (
        editor.root_sm_name
        or QInputDialog.getText(
            editor, "State Machine Name", "Enter state machine name (optional):"
        )[0]
    )
    root = ET.Element("StateMachine")
    root.set("name", sm_name)
    root.set("outcomes", " ".join(editor.final_outcomes.keys()))
    if editor.start_state:
        root.set("start_state", editor.start_state)
    root_level_states = {
        name: node
        for name, node in editor.state_nodes.items()
        if not hasattr(node, "parent_container") or node.parent_container is None
    }
    save_states_to_xml(root, root_level_states)
    tree = ET.ElementTree(root)
    tree.write(
        file_path if file_path.lower().endswith(".xml") else file_path + ".xml",
        encoding="utf-8",
        xml_declaration=True,
        pretty_print=True,
    )


def save_states_to_xml(parent_elem: ET.Element, state_nodes_dict: dict) -> None:
    """Recursively save states to XML."""
    for state_node in state_nodes_dict.values():
        if state_node.is_concurrence:
            save_container(parent_elem, state_node, "Concurrence")
        elif state_node.is_state_machine:
            save_container(parent_elem, state_node, "StateMachine")
        else:
            tag = (
                "State" if state_node.plugin_info.plugin_type != "xml" else "StateMachine"
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
                add_remappings(state_elem, state_node.remappings)
            save_transitions(state_elem, state_node)


def save_transitions(parent_elem: ET.Element, state_node: "StateNode") -> None:
    """Save transitions for a state node."""
    for connection in state_node.connections:
        if connection.from_node == state_node:
            transition = ET.SubElement(parent_elem, "Transition")
            if connection.outcome:
                transition.set("from", connection.outcome)
            if connection.to_node and connection.to_node.name:
                transition.set("to", connection.to_node.name)
