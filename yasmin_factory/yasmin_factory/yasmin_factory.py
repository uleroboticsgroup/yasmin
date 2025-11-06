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

import importlib
import xml.etree.ElementTree as ET
from yasmin import State, Blackboard, StateMachine, Concurrence
from yasmin_pybind_bridge import CppStateFactory


class YasminFactory:

    def __init__(self) -> None:
        """
        Initializes the factory, setting up the C++ state factory
        """

        self._cpp_factory = CppStateFactory()

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

            # Handle parameters if any
            params = state_elem.attrib.get("parameters", "")
            if params:
                param_list = [param.strip() for param in params.split(",")]
                return state_class(*param_list)
            else:
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

        sm = StateMachine(outcomes=root.attrib.get("outcomes", "").split(" "))

        for child in root:

            transitions = {}
            remainings = {}

            for cchild in child:
                if cchild.tag == "Transition":
                    transitions[cchild.attrib["from"]] = cchild.attrib["to"]
                elif cchild.tag == "Remap":
                    remainings[cchild.attrib["from"]] = cchild.attrib["to"]

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
                remainings=remainings,
            )

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

        tree = ET.parse(xml_file)
        root = tree.getroot()

        if root.tag != "StateMachine":
            raise ValueError("Root element must be 'StateMachine'")

        return self.create_sm(root)

    def cleanup(self) -> None:
        """
        Explicitly cleanup all created C++ state states and the factory.
        This should be called before the plugin loader is destroyed to avoid
        class loader warnings.
        """
        self._cpp_factory.clear_states()

    def __del__(self) -> None:
        """
        Destructor that ensures proper cleanup of C++ objects.
        """
        try:
            self.cleanup()
        except:
            # Ignore errors during cleanup in destructor
            pass
