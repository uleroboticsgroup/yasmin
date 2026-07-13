# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import importlib

from typing import Dict

from ament_index_python import get_package_share_path
from lxml import etree as ET
import yasmin
from yasmin import Concurrence, JoinState, OrthogonalState, State, StateMachine
from yasmin.orthogonal_state import setup_default_gil_hooks
from yasmin_pybind_bridge import CppStateFactory

from yasmin_factory.type_utils import parse_key_value


class YasminFactory:

    def __init__(self) -> None:
        """
        Initializes the factory, setting up the C++ state factory
        and default GIL hooks for OrthogonalState threads.
        """

        self._cpp_factory = CppStateFactory()

        # Set up default GIL hooks for OrthogonalState so that threads
        # spawned by orthogonal regions can safely call into Python.
        setup_default_gil_hooks()

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

    def create_concurrence(
        self, conc_elem: ET.Element, base_dir: str = ""
    ) -> Concurrence:
        """
        Creates a concurrence from an XML element.
        Args:
            conc_elem (ET.Element): The XML element defining the concurrence.
            base_dir (str): Base directory for resolving relative file_path includes.
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
                states[child.attrib["name"]] = self.create_concurrence(child, base_dir)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "StateMachine":
                states[child.attrib["name"]] = self.create_sm(child, base_dir)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "JoinState":
                states[child.attrib["name"]] = self._create_join_state(child)
                parameter_mappings[child.attrib["name"]] = self._get_parameter_mappings(
                    child
                )

            elif child.tag == "OrthogonalState":
                states[child.attrib["name"]] = self._create_orthogonal_state(
                    child, base_dir
                )
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

            elif child.tag == "Outcome":
                outcome_name = child.attrib["to"]
                outcome_map[outcome_name] = {}

                for trans in child:
                    if trans.tag == "Transition":
                        state_name = trans.attrib["state"]
                        outcome = trans.attrib["outcome"]
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

    def _create_orthogonal_state(
        self, orth_elem: ET.Element, base_dir: str = ""
    ) -> OrthogonalState:
        default_outcome = orth_elem.attrib.get("default_outcome", "")

        outcome_map = {}
        region_names = set()
        for child in orth_elem:
            if child.tag == "OutcomeMap":
                outcome_name = child.attrib["outcome"]
                outcome_map[outcome_name] = {}
                for item in child:
                    if item.tag == "Item":
                        outcome_map[outcome_name][item.attrib["state"]] = item.attrib[
                            "outcome"
                        ]

            elif child.tag == "Outcome":
                outcome_name = child.attrib["to"]
                outcome_map[outcome_name] = {}
                for trans in child:
                    if trans.tag == "Transition":
                        state_name = trans.attrib["state"]
                        outcome = trans.attrib["outcome"]
                        outcome_map[outcome_name][state_name] = outcome

        ort = OrthogonalState(default_outcome, outcome_map)

        for child in orth_elem:
            if child.tag == "Region":
                region_name = child.attrib["name"]
                region_sm = self.create_sm(child, base_dir)
                region_sm.set_name(region_name)
                ort.add_region(region_name, region_sm)
                region_names.add(region_name)

        for outcome_name, state_map in outcome_map.items():
            for state_name in state_map:
                if state_name not in region_names:
                    yasmin.YASMIN_LOG_WARN(
                        f"OutcomeMap '{outcome_name}' references unknown region '{state_name}' "
                        f"in OrthogonalState"
                    )

        self._add_blackboard_keys(ort, orth_elem)
        self._add_parameters(ort, orth_elem)
        return ort

    def _create_join_state(self, join_elem: ET.Element) -> JoinState:
        sync_id = join_elem.attrib.get("sync_id", "")
        outcome = join_elem.attrib.get("outcome", "joined")
        return JoinState(sync_id, outcome)

    def create_sm(self, root: ET.Element, base_dir: str = "") -> StateMachine:
        """
        Recursively creates a state machine from an XML element.
        Args:
            root (ET.Element): The XML element defining the state machine.
            base_dir (str): Base directory for resolving relative file_path
                includes. When empty, relative file_path attributes cannot be
                resolved and will raise an error.
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
                    matches = []
                    for dirpath, _, files in os.walk(package_path):
                        if file_name in files:
                            matches.append(os.path.join(dirpath, file_name))
                    if len(matches) > 1:
                        yasmin.YASMIN_LOG_WARN(
                            f"Found {len(matches)} matches for '{file_name}' in "
                            f"package '{package}', using first match: {matches[0]}"
                        )
                    if matches:
                        file_path = matches[0]
                except (OSError, ValueError, KeyError):
                    file_path = ""

        if file_path:
            if not os.path.isabs(file_path):
                file_path = os.path.normpath(os.path.join(base_dir, file_path))

            # Prevent path traversal outside the base directory
            if base_dir:
                base_dir_real = os.path.realpath(base_dir)
                resolved = os.path.realpath(file_path)
                if (
                    not resolved.startswith(base_dir_real + os.sep)
                    and resolved != base_dir_real
                ):
                    raise ValueError(
                        f"File path '{file_path}' resolves outside the XML directory"
                    )
                file_path = resolved

            return self.create_sm_from_file(file_path)

        sm = StateMachine(outcomes=root.attrib.get("outcomes", "").split())
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
                state = self.create_concurrence(child, base_dir)

            elif child.tag == "StateMachine":
                state = self.create_sm(child, base_dir)

            elif child.tag == "OrthogonalState":
                state = self._create_orthogonal_state(child, base_dir)

            elif child.tag == "JoinState":
                state = self._create_join_state(child)

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

        tree = ET.parse(xml_file)
        root = tree.getroot()

        if root.tag != "StateMachine":
            raise ValueError("Root element must be 'StateMachine'")

        # Read the name of the state machine root if available
        fsm_name = root.attrib.get("name", "")

        # Create the state machine, threading the file's directory as base_dir
        # so that relative file_path includes resolve correctly even with
        # recursive create_sm_from_file calls.
        sm = self.create_sm(root, os.path.dirname(xml_file))
        sm.set_name(fsm_name)
        return sm

    def _add_parameters(self, owner, parent_elem: ET.Element) -> None:
        """Parse Param elements into state-local parameters."""
        for param_elem in parent_elem.findall("Param"):
            parameter_name = param_elem.attrib["name"]
            parameter_description = param_elem.attrib.get("description", "")
            default_type = param_elem.attrib.get("default_type", "str")
            default_value = param_elem.attrib.get("default_value")

            if default_value is not None:
                value = parse_key_value(default_value, default_type)
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
                    value = parse_key_value(default_value, default_type)
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
            value = parse_key_value(value_str, type_str)
            owner.add_input_key(key_name, key_description, value)
