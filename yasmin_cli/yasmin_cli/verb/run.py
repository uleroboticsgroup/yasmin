# Copyright (C) 2026 Maik Knof
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

from __future__ import annotations

import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Union

from yasmin_factory.type_utils import (
    format_default_value,
    normalize_type,
    parse_key_value,
)

from yasmin_cli.completer import (
    get_state_machine_input_keys,
    get_state_machine_parameters,
    is_state_machine_xml,
    run_input_completer,
    run_param_completer,
    strip_namespace,
    xml_file_completer,
)

INPUT_KEY_TYPES = {"in", "in/out"}


def _parse_assignments(values: List[str], assignment_kind: str) -> Dict[str, str]:
    result: Dict[str, str] = {}

    for value in values:
        if "=" not in value:
            raise ValueError(
                f"Invalid {assignment_kind} assignment '{value}'. Expected NAME=VALUE."
            )

        key, raw_value = value.split("=", 1)
        key = key.strip()

        if not key:
            raise ValueError(
                f"Invalid {assignment_kind} assignment '{value}'. Empty name is not allowed."
            )

        result[key] = raw_value

    return result


def _indent_xml(element: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "    "
    child_indent = "\n" + (level + 1) * "    "

    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent

        for child in element:
            _indent_xml(child, level + 1)

        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = indent

    if level and (not element.tail or not element.tail.strip()):
        element.tail = indent


def _find_input_key_map(root: ET.Element) -> Dict[str, ET.Element]:
    result: Dict[str, ET.Element] = {}

    for child in root:
        if strip_namespace(child.tag) != "Key":
            continue

        key_type = (child.attrib.get("type") or "").strip().lower()
        if key_type not in INPUT_KEY_TYPES:
            continue

        default_type = (child.attrib.get("default_type") or "").strip()
        if not default_type:
            continue

        key_name = child.attrib.get("name", "").strip()
        if not key_name:
            continue

        result[key_name] = child

    return result


def _find_parameter_map(root: ET.Element) -> Dict[str, ET.Element]:
    result: Dict[str, ET.Element] = {}

    for child in root:
        if strip_namespace(child.tag) != "Param":
            continue

        parameter_name = child.attrib.get("name", "").strip()
        if not parameter_name:
            continue

        result[parameter_name] = child

    return result


def _find_default_element(root: ET.Element, key_name: str) -> Union[ET.Element, None]:
    for child in root:
        if strip_namespace(child.tag) != "Default":
            continue
        if child.attrib.get("key", "").strip() == key_name:
            return child
    return None


def _inject_overrides_into_xml(
    xml_path: str,
    provided_inputs: Dict[str, str],
    provided_parameters: Dict[str, str],
) -> str:
    tree = ET.parse(xml_path)
    root = tree.getroot()

    if strip_namespace(root.tag) != "StateMachine":
        raise ValueError(f"Not a valid YASMIN state machine XML file: {xml_path}")

    input_key_map = _find_input_key_map(root)
    unknown_inputs = sorted(name for name in provided_inputs if name not in input_key_map)
    if unknown_inputs:
        raise ValueError(
            f"Unknown input keys for state machine '{xml_path}': {', '.join(unknown_inputs)}"
        )

    parameter_map = _find_parameter_map(root)
    unknown_parameters = sorted(
        name for name in provided_parameters if name not in parameter_map
    )
    if unknown_parameters:
        raise ValueError(
            f"Unknown parameters for state machine '{xml_path}': {', '.join(unknown_parameters)}"
        )

    for key_name, raw_value in provided_inputs.items():
        key_element = input_key_map[key_name]
        default_type = key_element.attrib.get("default_type", "").strip()
        typed_value = parse_key_value(raw_value, default_type)
        serialized_value = format_default_value(typed_value, default_type)

        key_element.set("default_value", serialized_value)
        key_element.set("default_type", normalize_type(default_type))

        default_element = _find_default_element(root, key_name)
        if default_element is not None:
            default_element.set("value", serialized_value)
            default_element.set("type", normalize_type(default_type))

    for parameter_name, raw_value in provided_parameters.items():
        parameter_element = parameter_map[parameter_name]
        default_type = parameter_element.attrib.get("default_type", "").strip() or "str"
        typed_value = parse_key_value(raw_value, default_type)
        serialized_value = format_default_value(typed_value, default_type)

        parameter_element.set("default_value", serialized_value)
        parameter_element.set("default_type", normalize_type(default_type))

    _indent_xml(root)
    return ET.tostring(root, encoding="unicode")


def run_factory_node(
    state_machine_file: str,
    disable_viewer_pub: bool = False,
    use_python: bool = False,
) -> int:
    executable = "yasmin_factory_node.py" if use_python else "yasmin_factory_node"

    command = [
        "ros2",
        "run",
        "yasmin_factory",
        executable,
        "--ros-args",
        "-p",
        f"state_machine_file:={state_machine_file}",
        "-p",
        f"enable_viewer_pub:={'false' if disable_viewer_pub else 'true'}",
    ]

    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


def add_run_verb(subparsers):
    parser = subparsers.add_parser(
        "run",
        help="Run a YASMIN state machine from an XML file",
        description="Run a YASMIN state machine from an XML file",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = xml_file_completer

    input_arg = parser.add_argument(
        "--input",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override an input key from the XML state machine, may be given multiple times",
    )
    input_arg.completer = run_input_completer

    param_arg = parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="PARAM=VALUE",
        help="Override a declared parameter from the XML state machine, may be given multiple times",
    )
    param_arg.completer = run_param_completer

    parser.add_argument(
        "--disable-viewer-pub",
        action="store_true",
        help="Disable FSM viewer publisher",
    )
    parser.add_argument(
        "--py",
        action="store_true",
        help="Use the Python factory node instead of the C++ factory node",
    )

    parser.set_defaults(main=_main_run)


def _main_run(args):
    xml_path = Path(args.state_machine_file)
    if not xml_path.is_file():
        print(f"File does not exist: {args.state_machine_file}")
        return 1

    if not is_state_machine_xml(xml_path):
        print(f"Not a valid YASMIN state machine XML file: {args.state_machine_file}")
        return 1

    try:
        provided_inputs = _parse_assignments(args.input, "input")
        provided_parameters = _parse_assignments(args.param, "parameter")
    except ValueError as exc:
        print(str(exc))
        return 1

    if not provided_inputs and not provided_parameters:
        return run_factory_node(
            state_machine_file=args.state_machine_file,
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
        )

    valid_input_names = {
        key.get("name", "")
        for key in get_state_machine_input_keys(args.state_machine_file)
    }
    unknown_inputs = sorted(
        name for name in provided_inputs if name not in valid_input_names
    )
    if unknown_inputs:
        print(
            f"Unknown input keys for state machine '{args.state_machine_file}': "
            f"{', '.join(unknown_inputs)}"
        )
        return 1

    valid_parameter_names = {
        parameter.get("name", "")
        for parameter in get_state_machine_parameters(args.state_machine_file)
    }
    unknown_parameters = sorted(
        name for name in provided_parameters if name not in valid_parameter_names
    )
    if unknown_parameters:
        print(
            f"Unknown parameters for state machine '{args.state_machine_file}': "
            f"{', '.join(unknown_parameters)}"
        )
        return 1

    try:
        xml_content = _inject_overrides_into_xml(
            args.state_machine_file,
            provided_inputs,
            provided_parameters,
        )
    except ValueError as exc:
        print(str(exc))
        return 1
    except ET.ParseError as exc:
        print(f"Failed to parse XML file '{args.state_machine_file}': {exc}")
        return 1

    with tempfile.TemporaryDirectory(prefix="yasmin_run_") as temp_dir:
        temp_path = Path(temp_dir) / xml_path.name
        temp_path.write_text(xml_content, encoding="utf-8")

        return run_factory_node(
            state_machine_file=temp_path.as_posix(),
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
        )
