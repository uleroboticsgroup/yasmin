#!/usr/bin/env python3

# Copyright (C) 2026 Maik Knof
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

from __future__ import annotations

import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from yasmin_cli.completer import (
    get_state_machine_input_keys,
    is_state_machine_xml,
    run_input_completer,
    strip_namespace,
    xml_file_completer,
)

INPUT_KEY_TYPES = {"IN", "IN/OUT"}


def _parse_input_assignments(values: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}

    for value in values:
        if "=" not in value:
            raise ValueError(f"Invalid input assignment '{value}'. Expected KEY=VALUE.")

        key, raw_value = value.split("=", 1)
        key = key.strip()

        if not key:
            raise ValueError(
                f"Invalid input assignment '{value}'. Empty key is not allowed."
            )

        result[key] = raw_value

    return result


def _normalize_type(type_name: str) -> str:
    normalized = (type_name or "").strip().lower()

    if normalized in ("str", "string"):
        return "str"
    if normalized in ("int", "integer"):
        return "int"
    if normalized in ("float", "double"):
        return "float"
    if normalized in ("bool", "boolean"):
        return "bool"

    raise ValueError(f"Unsupported default_type '{type_name}'")


def _convert_value(raw_value: str, type_name: str):
    normalized_type = _normalize_type(type_name)

    if normalized_type == "str":
        return raw_value

    if normalized_type == "int":
        try:
            return int(raw_value)
        except ValueError as exc:
            raise ValueError(f"Invalid value '{raw_value}' for type int") from exc

    if normalized_type == "float":
        try:
            return float(raw_value)
        except ValueError as exc:
            raise ValueError(f"Invalid value '{raw_value}' for type float") from exc

    if normalized_type == "bool":
        lowered = raw_value.strip().lower()
        if lowered in ("true", "1", "yes", "on"):
            return True
        if lowered in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"Invalid value '{raw_value}' for type bool")

    raise ValueError(f"Unsupported default_type '{type_name}'")


def _format_default_value(value, type_name: str) -> str:
    normalized_type = _normalize_type(type_name)

    if normalized_type == "bool":
        return "true" if value else "false"

    return str(value)


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


def _find_input_key_map(root: ET.Element) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}

    for child in root:
        if strip_namespace(child.tag) != "Key":
            continue

        key_type = (child.attrib.get("type") or "").strip().upper()
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


def _find_default_element(root: ET.Element, key_name: str) -> ET.Element | None:
    for child in root:
        if strip_namespace(child.tag) != "Default":
            continue
        if child.attrib.get("key", "").strip() == key_name:
            return child
    return None


def _inject_inputs_into_xml(xml_path: str, provided_inputs: dict[str, str]) -> str:
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

    for key_name, raw_value in provided_inputs.items():
        key_element = input_key_map[key_name]
        default_type = key_element.attrib.get("default_type", "").strip()
        typed_value = _convert_value(raw_value, default_type)
        serialized_value = _format_default_value(typed_value, default_type)

        key_element.set("default_value", serialized_value)
        key_element.set("default_type", _normalize_type(default_type))

        default_element = _find_default_element(root, key_name)
        if default_element is not None:
            default_element.set("value", serialized_value)
            default_element.set("type", _normalize_type(default_type))

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
        provided_inputs = _parse_input_assignments(args.input)
    except ValueError as exc:
        print(str(exc))
        return 1

    if not provided_inputs:
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

    try:
        xml_content = _inject_inputs_into_xml(args.state_machine_file, provided_inputs)
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
