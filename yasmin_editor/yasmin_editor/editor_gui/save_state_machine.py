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


def save_to_xml(editor: "YasminEditor", file_path: str) -> None:
    """Save the state machine to an XML file.

    Args:
        file_path: The path where the XML file will be saved.
    """
    sm_name = editor.root_sm_name
    final_outcomes = editor.final_outcomes
    start_state = editor.start_state
    state_nodes = editor.state_nodes

    if not sm_name:
        sm_name, ok = QInputDialog.getText(
            editor, "State Machine Name", "Enter state machine name (optional):"
        )
    else:
        ok = True

    root = ET.Element("StateMachine")
    root.set("name", sm_name)
    root.set("outcomes", " ".join(final_outcomes.keys()))

    if start_state:
        root.set("start_state", start_state)

    root_level_states = {
        name: node
        for name, node in state_nodes.items()
        if not hasattr(node, "parent_container") or node.parent_container is None
    }

    save_states_to_xml(root, root_level_states)
    if not file_path.lower().endswith(".xml"):
        file_path += ".xml"

    tree = ET.ElementTree(root)
    tree.write(file_path, encoding="utf-8", xml_declaration=True, pretty_print=True)


def save_states_to_xml(parent_elem: ET.Element, state_nodes_dict: dict) -> None:
    """Recursively save states and their children to XML.

    Args:
        parent_elem: The parent XML element.
        state_nodes_dict: Dictionary of state names to state nodes.
    """
    for state_name, state_node in state_nodes_dict.items():
        if state_node.is_concurrence:
            # Concurrence state
            cc_elem = ET.SubElement(parent_elem, "Concurrence")
            cc_elem.set("name", state_name)

            if hasattr(state_node, "default_outcome") and state_node.default_outcome:
                cc_elem.set("default_outcome", state_node.default_outcome)

            if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                cc_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

            if state_node.remappings:
                for old_key, new_key in state_node.remappings.items():
                    if old_key and new_key:
                        remap_elem = ET.SubElement(cc_elem, "Remap")
                        remap_elem.set("old", old_key)
                        remap_elem.set("new", new_key)

            if hasattr(state_node, "child_states") and state_node.child_states:
                save_states_to_xml(cc_elem, state_node.child_states)

            save_transitions(cc_elem, state_node)

            if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                for outcome_node in state_node.final_outcomes.values():
                    save_transitions(cc_elem, outcome_node)

        elif state_node.is_state_machine:
            sm_elem = ET.SubElement(parent_elem, "StateMachine")
            sm_elem.set("name", state_name)

            if hasattr(state_node, "start_state") and state_node.start_state:
                sm_elem.set("start_state", state_node.start_state)

            if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                sm_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

            if state_node.remappings:
                for old_key, new_key in state_node.remappings.items():
                    if old_key and new_key:
                        remap_elem = ET.SubElement(sm_elem, "Remap")
                        remap_elem.set("old", old_key)
                        remap_elem.set("new", new_key)

            if hasattr(state_node, "child_states") and state_node.child_states:
                save_states_to_xml(sm_elem, state_node.child_states)

            save_transitions(sm_elem, state_node)

            # Add transitions from final outcomes inside this state machine
            if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                for outcome_node in state_node.final_outcomes.values():
                    save_transitions(sm_elem, outcome_node)

        else:
            if state_node.plugin_info.plugin_type != "xml":
                state_elem = ET.SubElement(parent_elem, "State")
            else:
                state_elem = ET.SubElement(parent_elem, "StateMachine")
            state_elem.set("name", state_name)

            if state_node.plugin_info:
                if state_node.plugin_info.plugin_type == "python":
                    state_elem.set("type", "py")
                    if state_node.plugin_info.module:
                        state_elem.set("module", state_node.plugin_info.module)
                    if state_node.plugin_info.class_name:
                        state_elem.set("class", state_node.plugin_info.class_name)
                elif state_node.plugin_info.plugin_type == "cpp":
                    state_elem.set("type", "cpp")
                    if state_node.plugin_info.class_name:
                        state_elem.set("class", state_node.plugin_info.class_name)
                elif state_node.plugin_info.plugin_type == "xml":
                    state_elem.set("type", "xml")
                    if state_node.plugin_info.file_name:
                        state_elem.set("file_name", state_node.plugin_info.file_name)
                        state_elem.set("package", state_node.plugin_info.package_name)

            if state_node.remappings:
                for old_key, new_key in state_node.remappings.items():
                    if old_key and new_key:
                        remap_elem = ET.SubElement(state_elem, "Remap")
                        remap_elem.set("old", old_key)
                        remap_elem.set("new", new_key)

            save_transitions(state_elem, state_node)


def save_transitions(parent_elem: ET.Element, state_node: "StateNode") -> None:
    """Save transitions for a state node.

    Args:
        parent_elem: The parent XML element.
        state_node: The state node whose transitions to save.
    """
    for connection in state_node.connections:
        if connection.from_node == state_node:
            transition = ET.SubElement(parent_elem, "Transition")
            if connection.outcome:
                transition.set("from", connection.outcome)
            if connection.to_node and connection.to_node.name:
                transition.set("to", connection.to_node.name)
