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

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from yasmin_cli.completer import (
    find_plugin,
    input_completer,
    plugin_id,
    test_plugin_completer,
)
from yasmin_cli.verb.run import run_factory_node


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
    normalized = (type_name or "str").strip().lower()

    if normalized in ("str", "string"):
        return "str"
    if normalized in ("int", "integer"):
        return "int"
    if normalized in ("float", "double"):
        return "float"
    if normalized in ("bool", "boolean"):
        return "bool"

    return "str"


def _infer_value_type(raw_value: str) -> str:
    value = raw_value.strip()

    if value.lower() in ("true", "false", "1", "0"):
        return "bool"

    try:
        int(value)
        return "int"
    except ValueError:
        pass

    try:
        float(value)
        return "float"
    except ValueError:
        pass

    return "str"


def _get_input_meta(plugin, key_name: str) -> dict | None:
    for key in plugin.input_keys:
        if key.get("name", "") == key_name:
            return key
    return None


def _resolve_input_type(plugin, key_name: str, raw_value: str) -> str:
    key_meta = _get_input_meta(plugin, key_name)
    if key_meta is None:
        return _infer_value_type(raw_value)

    default_type = key_meta.get("default_value_type", "")
    if default_type:
        return _normalize_type(default_type)

    if key_meta.get("has_default", False):
        default_value = key_meta.get("default_value")
        if isinstance(default_value, bool):
            return "bool"
        if isinstance(default_value, int) and not isinstance(default_value, bool):
            return "int"
        if isinstance(default_value, float):
            return "float"
        if isinstance(default_value, str):
            return "str"

    return _infer_value_type(raw_value)


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


def _build_test_xml(plugin, provided_inputs: dict[str, str]) -> str:
    outcomes = [outcome for outcome in plugin.outcomes if outcome]
    if not outcomes:
        outcomes = ["done"]

    root = ET.Element(
        "StateMachine",
        {
            "name": "TestStateMachine",
            "outcomes": " ".join(outcomes),
        },
    )

    for key_name, raw_value in provided_inputs.items():
        key_meta = _get_input_meta(plugin, key_name)
        attributes = {
            "key": key_name,
            "value": raw_value,
            "type": _resolve_input_type(plugin, key_name, raw_value),
        }

        if key_meta is not None:
            description = key_meta.get("description", "")
            if description:
                attributes["description"] = description

        ET.SubElement(root, "Default", attributes)

    state_attributes = {
        "name": "TestState",
        "class": plugin.class_name,
    }

    if plugin.plugin_type == "python":
        state_attributes["type"] = "py"
        state_attributes["module"] = plugin.module
    elif plugin.plugin_type == "cpp":
        state_attributes["type"] = "cpp"
    else:
        raise ValueError(f"Unsupported plugin type for test verb: {plugin.plugin_type}")

    state = ET.SubElement(root, "State", state_attributes)

    for key in plugin.input_keys:
        key_name = key.get("name", "")
        if not key_name:
            continue

        ET.SubElement(
            state,
            "Remap",
            {
                "old": key_name,
                "new": key_name,
            },
        )

    for outcome in outcomes:
        ET.SubElement(
            state,
            "Transition",
            {
                "from": outcome,
                "to": outcome,
            },
        )

    _indent_xml(root)
    return ET.tostring(root, encoding="unicode")


def _print_plugin_input_summary(plugin, provided_inputs: dict[str, str]) -> None:
    if not plugin.input_keys:
        return

    print("Inputs:")
    for key in plugin.input_keys:
        name = key.get("name", "")
        description = key.get("description", "")
        has_default = key.get("has_default", False)
        default_value = key.get("default_value")

        line = f"  - {name}"
        if name in provided_inputs:
            line += f" = {provided_inputs[name]!r}"
        elif has_default:
            line += f" [plugin-default={default_value!r}]"
        else:
            line += " [not set]"

        print(line)

        if description:
            print(f"      {description}")


def add_test_verb(subparsers):
    parser = subparsers.add_parser(
        "test",
        help="Run one YASMIN state in an isolated temporary state machine",
        description="Create a temporary one-state XML and run it through yasmin_factory",
    )

    plugin_arg = parser.add_argument(
        "plugin_id",
        help="Plugin id of the state to test",
    )
    plugin_arg.completer = test_plugin_completer

    input_arg = parser.add_argument(
        "--input",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Input value for the selected state, may be given multiple times",
    )
    input_arg.completer = input_completer

    parser.add_argument(
        "--disable_viewer_pub",
        action="store_true",
        help="Disable FSM viewer publisher",
    )
    parser.add_argument(
        "--py",
        action="store_true",
        help="Use the Python factory node instead of the C++ factory node",
    )
    parser.add_argument(
        "--keep-temp-file",
        action="store_true",
        help="Keep the generated temporary XML file in the current directory",
    )
    parser.add_argument(
        "--print-xml",
        action="store_true",
        help="Print the generated temporary XML before execution",
    )

    parser.set_defaults(main=_main_test)


def _main_test(args):
    plugin = find_plugin(args.plugin_id, include_xml=False)
    if plugin is None:
        print(f"Plugin not found: {args.plugin_id}")
        return 1

    if plugin.plugin_type not in ("cpp", "python"):
        print(
            f"Unsupported plugin type '{plugin.plugin_type}' for test verb. "
            "Currently only cpp and python states are supported."
        )
        return 1

    try:
        provided_inputs = _parse_input_assignments(args.input)
    except ValueError as exc:
        print(str(exc))
        return 1

    valid_input_names = {key.get("name", "") for key in plugin.input_keys}
    unknown_inputs = sorted(
        name for name in provided_inputs if name not in valid_input_names
    )
    if unknown_inputs:
        print(
            f"Unknown input keys for plugin '{args.plugin_id}': "
            f"{', '.join(unknown_inputs)}"
        )
        return 1

    xml_content = _build_test_xml(plugin, provided_inputs)

    if args.print_xml:
        print(xml_content)

    if provided_inputs:
        _print_plugin_input_summary(plugin, provided_inputs)

    if args.keep_temp_file:
        safe_name = plugin_id(plugin).replace("/", "_").replace(".", "_")
        temp_path = Path.cwd() / f"yasmin_test_{safe_name}.xml"
        temp_path.write_text(xml_content, encoding="utf-8")
        return run_factory_node(
            state_machine_file=temp_path.as_posix(),
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
        )

    with tempfile.TemporaryDirectory(prefix="yasmin_test_") as temp_dir:
        temp_path = Path(temp_dir) / "test_state_machine.xml"
        temp_path.write_text(xml_content, encoding="utf-8")

        return run_factory_node(
            state_machine_file=temp_path.as_posix(),
            disable_viewer_pub=args.disable_viewer_pub,
            use_python=args.py,
        )
