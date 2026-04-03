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
import json
import os

from typing import Dict

from ament_index_python import get_package_share_path
from lxml import etree as ET
from yasmin import Concurrence, State, StateMachine
from yasmin_pybind_bridge import CppStateFactory


class YasminFactory:

    def __init__(self) -> None:
        """
        Initializes the factory, setting up the C++ state factory
        """

        self._cpp_factory = CppStateFactory()
        self._xml_path: str = ""

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
            state = state_class()

        elif state_type == "cpp":
            state = self._cpp_factory.create(class_name)

        else:
            raise ValueError(f"Unknown state type: {state_type}")

        self._add_blackboard_keys(state, state_elem)
        self._add_parameters(state, state_elem)
        return state

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
        parameter_mappings = {}

        for child in conc_elem:
            if child.tag == "State":
                states[child.attrib["name"]] = self.create_state(child)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "Concurrence":
                states[child.attrib["name"]] = self.create_concurrence(child)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "StateMachine":
                states[child.attrib["name"]] = self.create_sm(child)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "OutcomeMap":
                outcome_name = child.attrib["outcome"]
                outcome_map[outcome_name] = {}

                for item in child:
                    if item.tag == "Item":
                        state_name = item.attrib["state"]
                        outcome = item.attrib["outcome"]
                        outcome_map[outcome_name][state_name] = outcome

        concurrence = Concurrence(
            states=states,
            outcome_map=outcome_map,
            default_outcome=default_outcome,
            parameter_mappings=parameter_mappings,
        )

        self._add_blackboard_keys(concurrence, conc_elem)
        self._add_parameters(concurrence, conc_elem)

        for outcome_elem in conc_elem.findall("FinalOutcome"):
            outcome_name = outcome_elem.attrib["name"]
            outcome_description = outcome_elem.attrib.get("description", "")

            if outcome_description:
                concurrence.set_outcome_description(outcome_name, outcome_description)

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
                    for dirpath, _, files in os.walk(package_path):
                        if file_name in files:
                            file_path = os.path.join(dirpath, file_name)
                            break
                except Exception:
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
            parameter_mappings = {}

            for cchild in child:
                if cchild.tag == "Transition":
                    transitions[cchild.attrib["from"]] = cchild.attrib["to"]
                elif cchild.tag == "Remap":
                    remappings[cchild.attrib["old"]] = cchild.attrib["new"]
                elif cchild.tag == "ParamRemap":
                    parameter_mappings[cchild.attrib["old"]] = cchild.attrib["new"]

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
                parameter_mappings=parameter_mappings,
            )

        if set_start_state:
            sm.set_start_state(set_start_state)

        description = root.attrib.get("description", "")
        if description:
            sm.set_description(description)

        for outcome_elem in root.findall("FinalOutcome"):
            outcome_name = outcome_elem.attrib["name"]
            outcome_description = outcome_elem.attrib.get("description", "")

            if outcome_description:
                sm.set_outcome_description(outcome_name, outcome_description)

        self._add_blackboard_keys(sm, root)
        self._add_parameters(sm, root)
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

    def _parse_key_value(self, value_str: str, type_str: str):
        """Parse a default value from XML using the declared metadata type.

        Scalar values keep the historic plain-string representation. The new
        homogeneous list and dict types use JSON so values can be written once
        and parsed consistently by both the Python and the C++ factory.
        """
        normalized_type = self._normalize_value_type(type_str)

        if normalized_type == "str":
            return value_str
        if normalized_type == "int":
            return int(value_str)
        if normalized_type == "float":
            return float(value_str)
        if normalized_type == "bool":
            return self._parse_bool_value(value_str)
        if normalized_type.startswith("list["):
            return self._parse_list_value(value_str, normalized_type)
        if normalized_type.startswith("dict["):
            return self._parse_dict_value(value_str, normalized_type)

        raise ValueError(f"Unsupported default_type '{type_str}'")

    def _normalize_value_type(self, type_str: str) -> str:
        """Normalize XML value type aliases into one canonical representation."""
        normalized = (type_str or "str").strip().lower().replace(" ", "")

        alias_map = {
            "string": "str",
            "double": "float",
            "boolean": "bool",
            "list[string]": "list[str]",
            "list[double]": "list[float]",
            "list[boolean]": "list[bool]",
            "dict[string,str]": "dict[str,str]",
            "dict[str,string]": "dict[str,str]",
            "dict[string,string]": "dict[str,str]",
            "dict[string,int]": "dict[str,int]",
            "dict[string,integer]": "dict[str,int]",
            "dict[str,integer]": "dict[str,int]",
            "dict[string,float]": "dict[str,float]",
            "dict[string,double]": "dict[str,float]",
            "dict[str,double]": "dict[str,float]",
            "dict[string,bool]": "dict[str,bool]",
            "dict[string,boolean]": "dict[str,bool]",
            "dict[str,boolean]": "dict[str,bool]",
        }

        if normalized in alias_map:
            return alias_map[normalized]

        return normalized

    def _parse_bool_value(self, value_str: str) -> bool:
        """Parse a scalar boolean default value."""
        normalized = value_str.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"Invalid boolean default value '{value_str}'")

    def _load_json_value(self, value_str: str):
        """Load a JSON encoded list or dictionary value."""
        try:
            return json.loads(value_str)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"Invalid JSON default value '{value_str}' for container type"
            ) from exc

    def _parse_list_value(self, value_str: str, normalized_type: str):
        """Parse a homogeneous list default value from JSON."""
        value = self._load_json_value(value_str)
        if not isinstance(value, list):
            raise ValueError(
                f"Default value '{value_str}' must decode to a JSON list for type {normalized_type}"
            )

        if normalized_type == "list[str]":
            if not all(isinstance(item, str) for item in value):
                raise ValueError(f"Type {normalized_type} expects only string entries")
            return value

        if normalized_type == "list[int]":
            if not all(
                isinstance(item, int) and not isinstance(item, bool) for item in value
            ):
                raise ValueError(f"Type {normalized_type} expects only integer entries")
            return value

        if normalized_type == "list[float]":
            if not all(
                isinstance(item, (int, float)) and not isinstance(item, bool)
                for item in value
            ):
                raise ValueError(f"Type {normalized_type} expects only numeric entries")
            return [float(item) for item in value]

        if normalized_type == "list[bool]":
            if not all(isinstance(item, bool) for item in value):
                raise ValueError(f"Type {normalized_type} expects only boolean entries")
            return value

        raise ValueError(f"Unsupported list default type '{normalized_type}'")

    def _parse_dict_value(self, value_str: str, normalized_type: str):
        """Parse a homogeneous dict[str, T] default value from JSON."""
        value = self._load_json_value(value_str)
        if not isinstance(value, dict):
            raise ValueError(
                f"Default value '{value_str}' must decode to a JSON object for type {normalized_type}"
            )

        if not all(isinstance(key, str) for key in value.keys()):
            raise ValueError(f"Type {normalized_type} expects only string keys")

        if normalized_type == "dict[str,str]":
            if not all(isinstance(item, str) for item in value.values()):
                raise ValueError(f"Type {normalized_type} expects only string values")
            return value

        if normalized_type == "dict[str,int]":
            if not all(
                isinstance(item, int) and not isinstance(item, bool)
                for item in value.values()
            ):
                raise ValueError(f"Type {normalized_type} expects only integer values")
            return value

        if normalized_type == "dict[str,float]":
            if not all(
                isinstance(item, (int, float)) and not isinstance(item, bool)
                for item in value.values()
            ):
                raise ValueError(f"Type {normalized_type} expects only numeric values")
            return {key: float(item) for key, item in value.items()}

        if normalized_type == "dict[str,bool]":
            if not all(isinstance(item, bool) for item in value.values()):
                raise ValueError(f"Type {normalized_type} expects only boolean values")
            return value

        raise ValueError(f"Unsupported dict default type '{normalized_type}'")

    def _add_parameters(self, owner, parent_elem: ET.Element) -> None:
        """Parse Param elements into state-local parameters."""
        for param_elem in parent_elem.findall("Param"):
            parameter_name = param_elem.attrib["name"]
            parameter_description = param_elem.attrib.get("description", "")
            default_type = param_elem.attrib.get("default_type", "str")
            default_value = param_elem.attrib.get("default_value")

            if default_value is not None:
                value = self._parse_key_value(default_value, default_type)
                owner.declare_parameter(parameter_name, parameter_description, value)
            elif parameter_description:
                owner.declare_parameter(parameter_name, parameter_description)
            else:
                owner.declare_parameter(parameter_name)

    def _get_parameter_mappings(self, parent_elem: ET.Element) -> Dict[str, str]:
        parameter_mappings = {}
        for remap_elem in parent_elem.findall("ParamRemap"):
            parameter_mappings[remap_elem.attrib["old"]] = remap_elem.attrib["new"]
        return parameter_mappings

    def _add_parameters(self, owner, parent_elem: ET.Element) -> None:
        """Parse Param elements into state-local parameters."""
        for param_elem in parent_elem.findall("Param"):
            parameter_name = param_elem.attrib["name"]
            parameter_description = param_elem.attrib.get("description", "")
            default_type = param_elem.attrib.get("default_type", "str")
            default_value = param_elem.attrib.get("default_value")

            if default_value is not None:
                value = self._parse_key_value(default_value, default_type)
                owner.declare_parameter(parameter_name, parameter_description, value)
            elif parameter_description:
                owner.declare_parameter(parameter_name, parameter_description)
            else:
                owner.declare_parameter(parameter_name)

    def _get_parameter_mappings(self, parent_elem: ET.Element) -> Dict[str, str]:
        parameter_mappings = {}
        for remap_elem in parent_elem.findall("ParamRemap"):
            parameter_mappings[remap_elem.attrib["old"]] = remap_elem.attrib["new"]
        return parameter_mappings

    def _add_blackboard_keys(self, owner, parent_elem: ET.Element) -> None:
        """Parse new Key syntax and legacy Default syntax."""
        for key_elem in parent_elem.findall("Key"):
            key_name = key_elem.attrib["name"]
            key_usage = key_elem.attrib.get("type", "in").lower()
            key_description = key_elem.attrib.get("description", "")
            default_type = key_elem.attrib.get("default_type", "str")
            default_value = key_elem.attrib.get("default_value")

            if key_usage in ("in", "in/out"):
                if default_value is not None:
                    value = self._parse_key_value(default_value, default_type)
                    owner.add_input_key(key_name, key_description, value)
                else:
                    owner.add_input_key(key_name, key_description)

            if key_usage in ("out", "in/out"):
                owner.add_output_key(key_name, key_description)

        for def_elem in parent_elem.findall("Default"):
            key_name = def_elem.attrib["key"]
            value_str = def_elem.attrib["value"]
            type_str = def_elem.attrib.get("type", "str")
            key_description = def_elem.attrib.get("description", "")
            value = self._parse_key_value(value_str, type_str)
            owner.add_input_key(key_name, key_description, value)
