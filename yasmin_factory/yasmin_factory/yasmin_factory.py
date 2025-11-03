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
from yasmin_pybind_bridge import CppStateFactory, CppState


class _CppStateAdapter(State):
    """
    Adapter class to convert a C++ State instance into a Python State.
    """

    def __init__(self, cpp_state: CppState) -> None:
        """
        Initializes the adapter with a C++ State instance.
        """

        super().__init__(cpp_state.get_outcomes())
        self._cpp_state = cpp_state

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the C++ State instance with the provided blackboard.
        Args:
            blackboard (Blackboard): The blackboard to pass to the C++ state.
        Returns:
            str: The outcome returned by the C++ state.
        """
        return self._cpp_state(blackboard)

    def cleanup(self) -> None:
        """
        Explicitly cleanup the C++ state reference.
        """
        self._cpp_state = None

    def __del__(self) -> None:
        """
        Destructor that ensures proper cleanup of C++ state reference.
        """
        try:
            self.cleanup()
        except:
            # Ignore errors during cleanup in destructor
            pass


class YasminFactory:

    def __init__(self) -> None:
        """
        Initializes the plugin loader, setting up the C++ state factory if
        pybind is available.
        """

        self._cpp_factory = CppStateFactory()
        self._cpp_state_adapters = []  # Track created C++ state adapters

    def build_state(self, state_elem: ET.Element) -> State:
        """
        Loads a state from an XML element.
        Args:
            state_elem (ET.Element): The XML element defining the state.
        Returns:
            State: An instance of the loaded state.
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
            adapter = _CppStateAdapter(self._cpp_factory.create(class_name))
            self._cpp_state_adapters.append(adapter)  # Track the adapter
            return adapter

        else:
            raise ValueError(f"Unknown state type: {state_type}")

    def build_concurrence(self, conc_elem: ET.Element) -> Concurrence:
        """
        Loads a concurrence from an XML element.
        Args:
            conc_elem (ET.Element): The XML element defining the concurrence.
        Returns:
            Concurrence: An instance of the loaded concurrence.
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
                states[child.attrib["name"]] = self.build_state(child)

            elif child.tag == "Concurrence":
                states[child.attrib["name"]] = self.build_concurrence(child)

            elif child.tag == "StateMachine":
                states[child.attrib["name"]] = self.build_sm(child)

        concurrence = Concurrence(
            states=list(states.values()),
            outcome_map=outcome_map,
            default_outcome=default_outcome,
        )

        return concurrence

    def build_sm(self, root: ET.Element) -> StateMachine:
        """
        Recursively builds a state machine from an XML element.
        Args:
            root (ET.Element): The XML element defining the state machine.
        Returns:
            StateMachine: An instance of the built state machine.
        Raises:
            ValueError: If the XML structure is invalid.
        """

        sm = StateMachine(outcomes=root.attrib.get("outcomes", "").split(" "))

        for child in root:

            transitions = {}
            for cchild in child:
                if cchild.tag == "Transition":
                    transitions[cchild.attrib["from"]] = cchild.attrib["to"]

            if child.tag == "State":
                state = self.build_state(child)

            elif child.tag == "Concurrence":
                state = self.build_concurrence(child)

            elif child.tag == "StateMachine":
                state = self.build_sm(child)

            sm.add_state(
                child.attrib["name"],
                state,
                transitions=transitions,
            )

        return sm

    def load_sm(self, xml_file: str) -> StateMachine:
        """
        Loads a state machine from an XML file.
        Args:
            xml_file (str): Path to the XML file defining the state machine.
        Returns:
            StateMachine: An instance of the loaded state machine.
        Raises:
            ValueError: If the XML structure is invalid.
        """

        tree = ET.parse(xml_file)
        root = tree.getroot()

        if root.tag != "StateMachine":
            raise ValueError("Root element must be 'StateMachine'")

        return self.build_sm(root)

    def cleanup(self) -> None:
        """
        Explicitly cleanup all created C++ state adapters and the factory.
        This should be called before the plugin loader is destroyed to avoid
        class loader warnings.
        """
        # Cleanup all tracked C++ state adapters
        for adapter in self._cpp_state_adapters:
            adapter.cleanup()

        # Clear references to C++ state adapters
        self._cpp_state_adapters.clear()

        # Explicitly delete the C++ factory to trigger proper cleanup
        self._cpp_factory = None

    def __del__(self) -> None:
        """
        Destructor that ensures proper cleanup of C++ objects.
        """
        try:
            self.cleanup()
        except:
            # Ignore errors during cleanup in destructor
            pass
