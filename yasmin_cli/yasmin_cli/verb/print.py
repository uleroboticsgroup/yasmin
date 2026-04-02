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

from pathlib import Path
from xml.etree import ElementTree as ET

from yasmin_editor.io import model_from_xml
from yasmin_editor.model import validate_model

from yasmin_cli.completer import is_state_machine_xml, strip_namespace, xml_file_completer

CONTAINER_TAGS = {"StateMachine", "Concurrence"}
STATE_TAGS = {"State", "StateMachine", "Concurrence"}


def add_print_verb(subparsers):
    parser = subparsers.add_parser(
        "print",
        help="Print a YASMIN state machine from an XML file",
        description="Print a YASMIN state machine from an XML file",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = xml_file_completer

    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only print the validation result",
    )

    parser.set_defaults(main=_main_print)


def _format_name_list(values: list[str]) -> str:
    return f"[{', '.join(values)}]" if values else "[]"


def _find_immediate_children(element: ET.Element, child_tag: str) -> list[ET.Element]:
    return [child for child in element if strip_namespace(child.tag) == child_tag]


def _collect_declared_params(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "Param")


def _collect_declared_keys(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "Key")


def _collect_param_remaps(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "ParamRemap")


def _collect_remaps(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "Remap")


def _collect_transitions(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "Transition")


def _collect_final_outcomes(element: ET.Element) -> list[ET.Element]:
    return _find_immediate_children(element, "FinalOutcome")


def _collect_state_children(element: ET.Element) -> list[ET.Element]:
    return [child for child in element if strip_namespace(child.tag) in STATE_TAGS]


def _element_outcomes(element: ET.Element) -> list[str]:
    outcomes = element.attrib.get("outcomes", "")
    return [outcome for outcome in outcomes.split() if outcome]


def _format_param_line(param_elem: ET.Element) -> str:
    parameter_name = param_elem.attrib.get("name", "")
    description = param_elem.attrib.get("description", "")
    default_value = param_elem.attrib.get("default_value")
    default_type = param_elem.attrib.get("default_type")

    line = f"- {parameter_name}"
    metadata_parts: list[str] = []

    if default_value is not None:
        metadata_parts.append(f"default='{default_value}'")
    if default_type:
        metadata_parts.append(f"type={default_type}")

    if metadata_parts:
        line += f" [{', '.join(metadata_parts)}]"
    if description:
        line += f": {description}"

    return line


def _format_key_line(key_elem: ET.Element) -> str:
    key_name = key_elem.attrib.get("name", "")
    key_type = key_elem.attrib.get("type", "")
    description = key_elem.attrib.get("description", "")
    default_value = key_elem.attrib.get("default_value")
    default_type = key_elem.attrib.get("default_type")

    line = f"- {key_name}"
    metadata_parts: list[str] = []

    if key_type:
        metadata_parts.append(f"type={key_type}")
    if default_value is not None:
        metadata_parts.append(f"default='{default_value}'")
    if default_type:
        metadata_parts.append(f"default_type={default_type}")

    if metadata_parts:
        line += f" [{', '.join(metadata_parts)}]"
    if description:
        line += f": {description}"

    return line


def _format_container_header(element: ET.Element) -> str:
    tag = strip_namespace(element.tag)
    name = element.attrib.get("name", "")
    outcomes = _element_outcomes(element)
    params = [param.attrib.get("name", "") for param in _collect_declared_params(element)]
    params = [param for param in params if param]
    keys = [key.attrib.get("name", "") for key in _collect_declared_keys(element)]
    keys = [key for key in keys if key]

    if tag == "StateMachine":
        header = f"StateMachine(name='{name}'"
        start_state = element.attrib.get("start_state", "")
        if start_state:
            header += f", start_state='{start_state}'"
        header += f", outcomes={_format_name_list(outcomes)}"
    else:
        default_outcome = element.attrib.get("default_outcome", "")
        header = f"Concurrence(name='{name}'"
        if default_outcome:
            header += f", default_outcome='{default_outcome}'"
        header += f", outcomes={_format_name_list(outcomes)}"

    if params:
        header += f", params={_format_name_list(params)}"
    if keys:
        header += f", keys={_format_name_list(keys)}"

    header += ")"
    return header


def _format_state_header(element: ET.Element) -> str:
    tag = strip_namespace(element.tag)
    if tag == "State":
        name = element.attrib.get("name", "")
        state_type = element.attrib.get("type", "")

        if state_type == "py":
            module = element.attrib.get("module", "")
            class_name = element.attrib.get("class", "")
            return f"{name} (type=py, module={module}, class={class_name})"

        class_name = element.attrib.get("class", "")
        return f"{name} (type={state_type or 'unknown'}, class={class_name})"

    return element.attrib.get("name", tag)


def _append_lines(lines: list[str], indent: int, title: str, values: list[str]) -> None:
    if not values:
        return

    lines.append(f"{'  ' * indent}{title}:")
    for value in values:
        lines.append(f"{'  ' * (indent + 1)}{value}")


def _render_state_tree(element: ET.Element, indent: int, lines: list[str]) -> None:
    tag = strip_namespace(element.tag)

    if tag in CONTAINER_TAGS:
        lines.append(f"{'  ' * indent}{_format_container_header(element)}")

        description = element.attrib.get("description", "")
        if description:
            lines.append(f"{'  ' * (indent + 1)}description: {description}")

        _append_lines(
            lines,
            indent + 1,
            "params",
            [_format_param_line(param) for param in _collect_declared_params(element)],
        )
        _append_lines(
            lines,
            indent + 1,
            "keys",
            [_format_key_line(key) for key in _collect_declared_keys(element)],
        )
        _append_lines(
            lines,
            indent + 1,
            "param remaps",
            [
                f"{remap.attrib.get('old', '')} -> {remap.attrib.get('new', '')}"
                for remap in _collect_param_remaps(element)
            ],
        )

        state_children = _collect_state_children(element)
        if state_children:
            lines.append(f"{'  ' * (indent + 1)}states:")
            for child in state_children:
                child_tag = strip_namespace(child.tag)
                if child_tag == "State":
                    lines.append(f"{'  ' * (indent + 2)}- {_format_state_header(child)}")
                    _render_state_tree(child, indent + 3, lines)
                else:
                    lines.append(
                        f"{'  ' * (indent + 2)}- {child.attrib.get('name', child_tag)}"
                    )
                    _render_state_tree(child, indent + 3, lines)

        _append_lines(
            lines,
            indent + 1,
            "final outcomes",
            [
                f"- {outcome.attrib.get('name', '')}: {outcome.attrib.get('description', '')}".rstrip(
                    ": "
                )
                for outcome in _collect_final_outcomes(element)
            ],
        )
        _append_lines(
            lines,
            indent + 1,
            "container transitions",
            [
                f"{transition.attrib.get('from', '')} -> {transition.attrib.get('to', '')}"
                for transition in _collect_transitions(element)
            ],
        )
        return

    description = element.attrib.get("description", "")
    if description:
        lines.append(f"{'  ' * indent}description: {description}")

    _append_lines(
        lines,
        indent,
        "param remaps",
        [
            f"{remap.attrib.get('old', '')} -> {remap.attrib.get('new', '')}"
            for remap in _collect_param_remaps(element)
        ],
    )
    _append_lines(
        lines,
        indent,
        "remaps",
        [
            f"{remap.attrib.get('old', '')} -> {remap.attrib.get('new', '')}"
            for remap in _collect_remaps(element)
        ],
    )
    _append_lines(
        lines,
        indent,
        "transitions",
        [
            f"{transition.attrib.get('from', '')} -> {transition.attrib.get('to', '')}"
            for transition in _collect_transitions(element)
        ],
    )


def _render_state_machine(root: ET.Element) -> str:
    lines: list[str] = []
    _render_state_tree(root, 0, lines)
    return "\n".join(lines)


def _main_print(args):
    xml_path = Path(args.state_machine_file)
    if not xml_path.is_file():
        print(f"File does not exist: {args.state_machine_file}")
        return 1

    if not is_state_machine_xml(xml_path):
        print(f"Not a valid YASMIN state machine XML file: {args.state_machine_file}")
        return 1

    try:
        model = model_from_xml(xml_path)
    except ET.ParseError as exc:
        print(f"Failed to parse XML file '{args.state_machine_file}': {exc}")
        return 1
    except ValueError as exc:
        print(str(exc))
        return 1

    validation_result = validate_model(model)

    if not validation_result.is_valid:
        print(validation_result)
        if not args.validate_only:
            print()

    if args.validate_only:
        if validation_result.is_valid:
            print(validation_result)
        return 0 if validation_result.is_valid else 1

    try:
        root = ET.parse(xml_path).getroot()
        print(_render_state_machine(root))
    except ET.ParseError as exc:
        print(f"Failed to parse XML file '{args.state_machine_file}': {exc}")
        return 1

    return 0 if validation_result.is_valid else 1
