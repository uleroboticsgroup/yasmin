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

import os
import importlib
from lxml import etree as ET
from yasmin import State, StateMachine, Concurrence
from yasmin_pybind_bridge import CppStateFactory
from ament_index_python import get_package_share_path


class YasminFactory:

    def __init__(self) -> None:
        """
        Initializes the factory, setting up the C++ state factory
        """

        self._cpp_factory = CppStateFactory()
        self._xml_path = ""

    def create_state(self, state_elem: ET.Element) -> State:
        """
        Creates a state from an XML element.
        Args:
            state_elem (ET.Element): The XML element defining the state.
        Returns:
            State: An instance of the created state.
        Raises:
            ValueError: If the state type is unknown or if required attributes
                        are missing.
        """

        state_type = state_elem.attrib.get("type", "py")
        class_name = state_elem.attrib["class"]

        if state_type == "py":
            module_name = state_elem.attrib["module"]
            module = importlib.import_module(module_name)
            state_class = getattr(module, class_name)
            return state_class()

        elif state_type == "cpp":
            return self._cpp_factory.create(class_name)

        else:
            raise ValueError(f"Unknown state type: {state_type}")

    def create_concurrence(self, conc_elem: ET.Element) -> Concurrence:
        """
        Creates a concurrence from an XML element.
        Args:
            conc_elem (ET.Element): The XML element defining the concurrence.
        Returns:
            Concurrence: An instance of the created concurrence.
        Raises:
            ValueError: If required attributes are missing.
        """

        default_outcome = conc_elem.attrib.get("default_outcome", "")

        states = {}
        outcome_map = {}

        for child in conc_elem:
            for cchild in child:
                if cchild.tag == "Outcome":
                    outcome_map[cchild.attrib["to"]] = {}
                    for ccchild in cchild:
                        if ccchild.tag == "Transition":
                            outcome_map[cchild.attrib["to"]][
                                states[ccchild.attrib["state"]]
                            ] = ccchild.attrib["outcome"]

            if child.tag == "State":
                states[child.attrib["name"]] = self.create_state(child)

            elif child.tag == "Concurrence":
                states[child.attrib["name"]] = self.create_concurrence(child)

            elif child.tag == "StateMachine":
                states[child.attrib["name"]] = self.create_sm(child)

        concurrence = Concurrence(
            states=list(states.values()),
            outcome_map=outcome_map,
            default_outcome=default_outcome,
        )

        return concurrence

    def create_sm(self, root: ET.Element) -> StateMachine:
        """
        Recursively creates a state machine from an XML element.
        Args:
            root (ET.Element): The XML element defining the state machine.
        Returns:
            StateMachine: An instance of the created state machine.
        Raises:
            ValueError: If the XML structure is invalid.
        """
        file_path = root.attrib.get("file_path", "")

        if not file_path:
            file_name = root.attrib.get("file_name", "")
            package = root.attrib.get("package", "")

            if file_name and package:
                try:
                    package_path = get_package_share_path(package)
                    file_path = ""
                    for root, dirs, files in os.walk(package_path):
                        if file_name in files:
                            file_path = os.path.join(root, file_name)
                            break
                except Exception as e:
                    file_path = ""

        if file_path:
            if not os.path.isabs(file_path):
                file_path = os.path.join(os.path.dirname(self._xml_path), file_path)

            return self.create_sm_from_file(file_path)

        sm = StateMachine(outcomes=root.attrib.get("outcomes", "").split(" "))
        set_start_state = root.attrib.get("start_state", "")

        for child in root:

            transitions = {}
            remappings = {}

            for cchild in child:
                if cchild.tag == "Transition":
                    transitions[cchild.attrib["from"]] = cchild.attrib["to"]
                elif cchild.tag == "Remap":
                    remappings[cchild.attrib["old"]] = cchild.attrib["new"]

            if child.tag == "State":
                state = self.create_state(child)

            elif child.tag == "Concurrence":
                state = self.create_concurrence(child)

            elif child.tag == "StateMachine":
                state = self.create_sm(child)

            else:
                continue

            sm.add_state(
                child.attrib["name"],
                state,
                transitions=transitions,
                remappings=remappings,
            )

        if set_start_state:
            sm.set_start_state(set_start_state)

        return sm

    def create_sm_from_file(self, xml_file: str) -> StateMachine:
        """
        Creates a state machine from an XML file.
        Args:
            xml_file (str): Path to the XML file defining the state machine.
        Returns:
            StateMachine: An instance of the created state machine.
        Raises:
            ValueError: If the XML structure is invalid.
        """

        self._xml_path = xml_file
        tree = ET.parse(xml_file)
        root = tree.getroot()

        if root.tag != "StateMachine":
            raise ValueError("Root element must be 'StateMachine'")

        # Read the name of the state machine root if available
        fsm_name = root.attrib.get("name", "")

        # Create the state machine
        sm = self.create_sm(root)
        sm.set_name(fsm_name)
        return sm
