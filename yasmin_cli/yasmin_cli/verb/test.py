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

import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from yasmin_cli.completer import (
    build_plugin_info,
    input_completer,
    parameter_completer,
    plugin_id,
    test_plugin_completer,
)
from yasmin_cli.verb.run import (
    _convert_value,
    _format_default_value,
    _normalize_type,
    run_factory_node,
)


def _parse_assignments(values: list[str], assignment_kind: str) -> dict[str, str]:
    result: dict[str, str] = {}

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


def _infer_python_value_type(value) -> str | None:
    """Infer the canonical XML type string from an already parsed Python value."""
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int) and not isinstance(value, bool):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        if not value:
            return None
        if all(isinstance(item, str) for item in value):
            return "list[str]"
        if all(isinstance(item, bool) for item in value):
            return "list[bool]"
        if all(isinstance(item, int) and not isinstance(item, bool) for item in value):
            return "list[int]"
        if all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            for item in value
        ):
            return "list[float]"
        return None
    if isinstance(value, dict):
        if not value or not all(isinstance(key, str) for key in value.keys()):
            return None
        values = list(value.values())
        if all(isinstance(item, str) for item in values):
            return "dict[str,str]"
        if all(isinstance(item, bool) for item in values):
            return "dict[str,bool]"
        if all(isinstance(item, int) and not isinstance(item, bool) for item in values):
            return "dict[str,int]"
        if all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            for item in values
        ):
            return "dict[str,float]"
        return None
    return None


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

    try:
        parsed_json = json.loads(value)
    except json.JSONDecodeError:
        return "str"

    inferred_type = _infer_python_value_type(parsed_json)
    return inferred_type or "str"


def _serialize_default_value(value, type_name: str) -> str:
    """Serialize plugin metadata defaults back into the XML default format."""
    return _format_default_value(value, type_name)


def _get_input_meta(plugin, key_name: str) -> dict | None:
    for key in plugin.input_keys:
        if key.get("name", "") == key_name:
            return key
    return None


def _get_parameter_meta(plugin, parameter_name: str) -> dict | None:
    for parameter in getattr(plugin, "parameters", []):
        if parameter.get("name", "") == parameter_name:
            return parameter
    return None


def _resolve_input_type(plugin, key_name: str, raw_value: str) -> str:
    key_meta = _get_input_meta(plugin, key_name)
    if key_meta is None:
        return _infer_value_type(raw_value)

    default_type = key_meta.get("default_value_type", "")
    if default_type:
        return _normalize_type(default_type)

    if key_meta.get("has_default", False):
        inferred_type = _infer_python_value_type(key_meta.get("default_value"))
        if inferred_type:
            return inferred_type

    return _infer_value_type(raw_value)


def _resolve_parameter_type(plugin, parameter_name: str, raw_value: str) -> str:
    parameter_meta = _get_parameter_meta(plugin, parameter_name)
    if parameter_meta is None:
        return _infer_value_type(raw_value)

    default_type = parameter_meta.get("default_value_type", "")
    if default_type:
        return _normalize_type(default_type)

    if parameter_meta.get("has_default", False):
        inferred_type = _infer_python_value_type(parameter_meta.get("default_value"))
        if inferred_type:
            return inferred_type

    return _infer_value_type(raw_value)


def _resolve_parameter_type(plugin, parameter_name: str, raw_value: str) -> str:
    parameter_meta = _get_parameter_meta(plugin, parameter_name)
    if parameter_meta is None:
        return _infer_value_type(raw_value)

    default_type = parameter_meta.get("default_value_type", "")
    if default_type:
        return _normalize_type(default_type)

    if parameter_meta.get("has_default", False):
        default_value = parameter_meta.get("default_value")
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


def _merge_plugin_keys(plugin) -> list[dict]:
    merged_keys: dict[str, dict] = {}

    for key in plugin.input_keys:
        name = key.get("name", "")
        if not name:
            continue

        merged_keys[name] = dict(key)
        merged_keys[name]["direction"] = "in"

    for key in plugin.output_keys:
        name = key.get("name", "")
        if not name:
            continue

        if name in merged_keys:
            merged_keys[name]["direction"] = "in/out"
        else:
            merged_keys[name] = dict(key)
            merged_keys[name]["direction"] = "out"

    return list(merged_keys.values())


def _build_test_xml(
    plugin,
    provided_inputs: dict[str, str],
    provided_parameters: dict[str, str],
) -> str:
    outcomes = [outcome for outcome in plugin.outcomes if outcome]
    if not outcomes:
        outcomes = ["done"]

    root = ET.Element(
        "StateMachine",
        {
            "name": "TestStateMachine",
            "outcomes": " ".join(outcomes),
            "start_state": "TestState",
        },
    )

    for key in _merge_plugin_keys(plugin):
        key_name = key.get("name", "")
        if not key_name:
            continue

        key_attributes = {
            "name": key_name,
            "type": key.get("direction", "in"),
        }

        description = key.get("description", "")
        if description:
            key_attributes["description"] = description

        if key_name in provided_inputs:
            resolved_type = _resolve_input_type(
                plugin, key_name, provided_inputs[key_name]
            )
            converted_value = _convert_value(provided_inputs[key_name], resolved_type)
            key_attributes["default_value"] = _serialize_default_value(
                converted_value, resolved_type
            )
            key_attributes["default_type"] = resolved_type
        elif key.get("has_default", False):
            resolved_type = _normalize_type(key.get("default_value_type", "str"))
            key_attributes["default_value"] = _serialize_default_value(
                key.get("default_value", ""), resolved_type
            )
            key_attributes["default_type"] = resolved_type
        elif key.get("default_value_type"):
            key_attributes["default_type"] = _normalize_type(
                key.get("default_value_type", "str")
            )

        ET.SubElement(root, "Key", key_attributes)

    declared_parameter_names: list[str] = []
    for parameter in getattr(plugin, "parameters", []):
        parameter_name = parameter.get("name", "")
        if not parameter_name:
            continue

        if parameter_name not in provided_parameters and not parameter.get(
            "has_default", False
        ):
            continue

        parameter_attributes = {
            "name": parameter_name,
            "default_type": _normalize_type(parameter.get("default_value_type", "str")),
        }

        description = parameter.get("description", "")
        if description:
            parameter_attributes["description"] = description

        if parameter_name in provided_parameters:
            resolved_type = _resolve_parameter_type(
                plugin,
                parameter_name,
                provided_parameters[parameter_name],
            )
            converted_value = _convert_value(
                provided_parameters[parameter_name], resolved_type
            )
            parameter_attributes["default_value"] = _serialize_default_value(
                converted_value, resolved_type
            )
            parameter_attributes["default_type"] = resolved_type
        elif parameter.get("has_default", False):
            resolved_type = _normalize_type(parameter.get("default_value_type", "str"))
            parameter_attributes["default_value"] = _serialize_default_value(
                parameter.get("default_value", ""), resolved_type
            )

        ET.SubElement(root, "Param", parameter_attributes)
        declared_parameter_names.append(parameter_name)

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

    for key in _merge_plugin_keys(plugin):
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

    for parameter_name in declared_parameter_names:
        ET.SubElement(
            state,
            "ParamRemap",
            {
                "old": parameter_name,
                "new": parameter_name,
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


def _print_plugin_parameter_summary(plugin, provided_parameters: dict[str, str]) -> None:
    parameters = getattr(plugin, "parameters", [])
    if not parameters:
        return

    print("Parameters:")
    for parameter in parameters:
        name = parameter.get("name", "")
        description = parameter.get("description", "")
        has_default = parameter.get("has_default", False)
        default_value = parameter.get("default_value")

        line = f"  - {name}"
        if name in provided_parameters:
            line += f" = {provided_parameters[name]!r}"
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

    param_arg = parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="PARAM=VALUE",
        help="Parameter override for the selected state, may be given multiple times",
    )
    param_arg.completer = parameter_completer

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
    plugin = build_plugin_info(args.plugin_id)
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
        provided_inputs = _parse_assignments(args.input, "input")
        provided_parameters = _parse_assignments(args.param, "parameter")
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

    valid_parameter_names = {
        parameter.get("name", "") for parameter in getattr(plugin, "parameters", [])
    }
    unknown_parameters = sorted(
        name for name in provided_parameters if name not in valid_parameter_names
    )
    if unknown_parameters:
        print(
            f"Unknown parameters for plugin '{args.plugin_id}': "
            f"{', '.join(unknown_parameters)}"
        )
        return 1

    xml_content = _build_test_xml(plugin, provided_inputs, provided_parameters)

    if args.print_xml:
        print(xml_content)

    if provided_inputs:
        _print_plugin_input_summary(plugin, provided_inputs)
    if provided_parameters:
        _print_plugin_parameter_summary(plugin, provided_parameters)

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
