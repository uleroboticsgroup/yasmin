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
"""XML conversion for the YASMIN editor model."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition


def model_to_xml(model: StateMachine, file_path: str | Path | None = None) -> str:
    """Serialize a state machine model to XML."""

    root = _state_machine_to_element(model, parent=None)
    _indent(root)
    xml_text = ET.tostring(root, encoding="utf-8", xml_declaration=True).decode("utf-8")

    if file_path is not None:
        Path(file_path).write_text(xml_text, encoding="utf-8")

    return xml_text


def model_from_xml(xml_input: str | Path) -> StateMachine:
    """Deserialize a state machine model from XML text or a file path."""

    if isinstance(xml_input, Path):
        root = ET.parse(xml_input).getroot()
    elif (
        isinstance(xml_input, str)
        and not xml_input.lstrip().startswith("<")
        and Path(xml_input).exists()
    ):
        root = ET.parse(xml_input).getroot()
    else:
        root = ET.fromstring(str(xml_input))

    if root.tag != "StateMachine":
        raise ValueError("The root XML element must be 'StateMachine'.")

    return _parse_state_machine_container(root)


def _state_machine_to_element(
    model: StateMachine,
    parent: StateMachine | Concurrence | None,
) -> ET.Element:
    element = ET.Element("StateMachine")
    element.set("name", model.name)

    if model.outcomes:
        element.set("outcomes", " ".join(outcome.name for outcome in model.outcomes))
    if model.start_state:
        element.set("start_state", model.start_state)
    if model.description:
        element.set("description", model.description)

    if parent is not None:
        position = parent.layout.get_state_position(model.name)
        if position is not None:
            element.set("x", f"{position.x:.2f}")
            element.set("y", f"{position.y:.2f}")

    for parameter in model.parameters:
        element.append(_parameter_to_element(parameter))

    for key in model.keys:
        element.append(_key_to_element(key))

    _append_parameter_remaps(element, model.parameter_mappings)
    _append_remaps(element, model.remappings)

    for text_block in model.text_blocks:
        element.append(_text_block_to_element(text_block))

    for state in model.states.values():
        element.append(_state_to_element(state, model))

    for outcome in model.outcomes:
        element.append(_final_outcome_to_element(outcome, model))

    if parent is None:
        _append_container_level_transitions(element, model)
    else:
        _append_owner_transitions(element, parent, model.name)

    return element


def _concurrence_to_element(
    model: Concurrence,
    parent: StateMachine | Concurrence | None,
) -> ET.Element:
    element = ET.Element("Concurrence")
    element.set("name", model.name)

    if model.outcomes:
        element.set("outcomes", " ".join(outcome.name for outcome in model.outcomes))
    if model.default_outcome:
        element.set("default_outcome", model.default_outcome)
    if model.description:
        element.set("description", model.description)

    if parent is not None:
        position = parent.layout.get_state_position(model.name)
        if position is not None:
            element.set("x", f"{position.x:.2f}")
            element.set("y", f"{position.y:.2f}")

    for parameter in model.parameters:
        element.append(_parameter_to_element(parameter))

    for key in model.keys:
        element.append(_key_to_element(key))

    _append_parameter_remaps(element, model.parameter_mappings)
    _append_remaps(element, model.remappings)

    for text_block in model.text_blocks:
        element.append(_text_block_to_element(text_block))

    for state in model.states.values():
        element.append(_state_to_element(state, model))

    for outcome in model.outcomes:
        element.append(_final_outcome_to_element(outcome, model))

    _append_concurrence_outcome_map(element, model)

    if parent is not None and isinstance(parent, StateMachine):
        _append_owner_transitions(element, parent, model.name)

    return element


def _state_to_element(state: State, parent: StateMachine | Concurrence) -> ET.Element:
    if isinstance(state, StateMachine):
        return _state_machine_to_element(state, parent=parent)
    if isinstance(state, Concurrence):
        return _concurrence_to_element(state, parent=parent)

    tag = "State" if state.state_type != "xml" else "StateMachine"
    element = ET.Element(tag)
    element.set("name", state.name)

    if state.state_type:
        element.set("type", state.state_type)
    if state.state_type == "py" and state.module:
        element.set("module", state.module)
    if state.class_name:
        element.set("class", state.class_name)
    if state.state_type == "xml":
        if state.file_name:
            element.set("file_name", state.file_name)
        if state.package_name:
            element.set("package", state.package_name)
    if state.description:
        element.set("description", state.description)

    position = parent.layout.get_state_position(state.name)
    if position is not None:
        element.set("x", f"{position.x:.2f}")
        element.set("y", f"{position.y:.2f}")

    for parameter in state.parameters:
        element.append(_parameter_to_element(parameter))

    _append_parameter_remaps(element, state.parameter_mappings)
    _append_remaps(element, state.remappings)
    _append_owner_transitions(element, parent, state.name)
    return element


def _text_block_to_element(text_block: TextBlock) -> ET.Element:
    """Serialize one free-form text block."""
    element = ET.Element("Text")
    element.set("x", f"{text_block.x:.2f}")
    element.set("y", f"{text_block.y:.2f}")
    element.set("content", _encode_text_content(text_block.content))
    return element


def _final_outcome_to_element(
    outcome: Outcome,
    parent: StateMachine | Concurrence,
) -> ET.Element:
    element = ET.Element("FinalOutcome")
    element.set("name", outcome.name)

    if outcome.description:
        element.set("description", outcome.description)

    position = parent.layout.get_outcome_position(outcome.name)
    if position is not None:
        element.set("x", f"{position.x:.2f}")
        element.set("y", f"{position.y:.2f}")

    if isinstance(parent, StateMachine):
        for transition in parent.transitions.get(outcome.name, []):
            element.append(_transition_to_element(transition))

    return element


def _transition_to_element(transition: Transition) -> ET.Element:
    element = ET.Element("Transition")
    element.set("from", transition.source_outcome)
    element.set("to", transition.target)
    return element


def _key_to_element(key: Key) -> ET.Element:
    element = ET.Element("Key")
    element.set("name", key.name)
    element.set("type", key.key_type)

    if key.description:
        element.set("description", key.description)
    if key.has_default:
        element.set("default_type", key.default_type)
        element.set(
            "default_value",
            "" if key.default_value is None else str(key.default_value),
        )

    return element


def _parameter_to_element(parameter: Parameter) -> ET.Element:
    element = ET.Element("Param")
    element.set("name", parameter.name)

    if parameter.description:
        element.set("description", parameter.description)
    if parameter.has_default:
        element.set("default_type", parameter.default_type)
        element.set(
            "default_value",
            "" if parameter.default_value is None else str(parameter.default_value),
        )

    return element


def _append_parameter_remaps(
    element: ET.Element,
    parameter_mappings: dict[str, str],
) -> None:
    for child_parameter, parent_parameter in parameter_mappings.items():
        if not child_parameter or not parent_parameter:
            continue
        remap = ET.SubElement(element, "ParamRemap")
        remap.set("old", child_parameter)
        remap.set("new", parent_parameter)


def _append_remaps(element: ET.Element, remappings: dict[str, str]) -> None:
    for old, new in remappings.items():
        if not old or not new:
            continue
        remap = ET.SubElement(element, "Remap")
        remap.set("old", old)
        remap.set("new", new)


def _append_owner_transitions(
    element: ET.Element,
    parent: StateMachine | Concurrence,
    owner_name: str,
) -> None:
    if isinstance(parent, Concurrence):
        return
    for transition in parent.transitions.get(owner_name, []):
        element.append(_transition_to_element(transition))


def _append_container_level_transitions(
    element: ET.Element,
    model: StateMachine,
) -> None:
    child_names = set(model.states.keys())
    child_names.update(outcome.name for outcome in model.outcomes)

    for owner_name, transition_list in model.transitions.items():
        if owner_name in child_names:
            continue
        for transition in transition_list:
            element.append(_transition_to_element(transition))


def _append_concurrence_outcome_map(
    element: ET.Element,
    model: Concurrence,
) -> None:
    for outcome_name, requirements in model.outcome_map.items():
        outcome_map_elem = ET.SubElement(element, "OutcomeMap")
        outcome_map_elem.set("outcome", outcome_name)

        for state_name, state_outcome in requirements.items():
            item_elem = ET.SubElement(outcome_map_elem, "Item")
            item_elem.set("state", state_name)
            item_elem.set("outcome", state_outcome)


def _parse_state_machine_container(element: ET.Element) -> StateMachine:
    model = StateMachine(
        name=element.get("name", ""),
        description=element.get("description", ""),
        start_state=element.get("start_state"),
    )

    model.parameters.extend(_parse_parameters(element.findall("Param")))
    model.keys.extend(_parse_keys(element.findall("Key")))
    model.parameter_mappings.update(_parse_parameter_remaps(element))
    model.remappings.update(_parse_remaps(element))
    _parse_state_machine_content(model, element)
    return model


def _parse_concurrence_container(element: ET.Element) -> Concurrence:
    model = Concurrence(
        name=element.get("name", ""),
        description=element.get("description", ""),
        default_outcome=element.get("default_outcome"),
    )

    model.parameters.extend(_parse_parameters(element.findall("Param")))
    model.keys.extend(_parse_keys(element.findall("Key")))
    model.parameter_mappings.update(_parse_parameter_remaps(element))
    model.remappings.update(_parse_remaps(element))
    _parse_concurrence_content(model, element)
    return model


def _parse_state_machine_content(
    model: StateMachine,
    element: ET.Element,
) -> None:
    outcome_names = element.get("outcomes", "").split()
    for outcome_name in outcome_names:
        model.add_outcome(Outcome(name=outcome_name))

    for child in element:
        if child.tag in {"Param", "Key", "ParamRemap", "Remap"}:
            continue

        if child.tag == "Text":
            model.add_text_block(_parse_text_block(child))
            continue

        if child.tag == "FinalOutcome":
            _merge_final_outcome(model, child)
            continue

        if child.tag == "Transition":
            model.add_transition(model.name, _parse_transition(child))
            continue

        if child.tag == "OutcomeMap":
            continue

        state = _parse_state_like(child)
        model.add_state(state)

        x = _parse_float(child.get("x"))
        y = _parse_float(child.get("y"))
        if x is not None and y is not None:
            model.layout.set_state_position(state.name, x, y)

        for transition_elem in child.findall("Transition"):
            model.add_transition(state.name, _parse_transition(transition_elem))


def _parse_concurrence_content(
    model: Concurrence,
    element: ET.Element,
) -> None:
    outcome_names = element.get("outcomes", "").split()
    for outcome_name in outcome_names:
        model.add_outcome(Outcome(name=outcome_name))

    for child in element:
        if child.tag in {"Param", "Key", "ParamRemap", "Remap"}:
            continue

        if child.tag == "Text":
            model.add_text_block(_parse_text_block(child))
            continue

        if child.tag == "FinalOutcome":
            _merge_final_outcome(model, child)
            continue

        if child.tag == "OutcomeMap":
            _parse_outcome_map(model, child)
            continue

        if child.tag == "Transition":
            continue

        state = _parse_state_like(child)
        model.add_state(state)

        x = _parse_float(child.get("x"))
        y = _parse_float(child.get("y"))
        if x is not None and y is not None:
            model.layout.set_state_position(state.name, x, y)


def _merge_final_outcome(
    model: StateMachine | Concurrence,
    element: ET.Element,
) -> None:
    outcome_name = element.get("name", "")
    if not outcome_name:
        return

    outcome = next((item for item in model.outcomes if item.name == outcome_name), None)
    if outcome is None:
        outcome = Outcome(name=outcome_name)
        model.add_outcome(outcome)

    outcome.description = element.get("description", outcome.description)

    x = _parse_float(element.get("x"))
    y = _parse_float(element.get("y"))
    if x is not None and y is not None:
        model.layout.set_outcome_position(outcome.name, x, y)

    if isinstance(model, StateMachine):
        for transition_elem in element.findall("Transition"):
            model.add_transition(outcome.name, _parse_transition(transition_elem))


def _parse_outcome_map(model: Concurrence, element: ET.Element) -> None:
    outcome_name = element.get("outcome", "")
    if not outcome_name:
        return

    for item in element.findall("Item"):
        state_name = item.get("state", "")
        state_outcome = item.get("outcome", "")
        if state_name and state_outcome:
            model.set_outcome_rule(outcome_name, state_name, state_outcome)


def _parse_state_like(element: ET.Element) -> State:
    if element.tag == "Concurrence":
        return _parse_concurrence_container(element)

    if element.tag == "StateMachine" and not element.get("file_name"):
        return _parse_state_machine_container(element)

    state = State(
        name=element.get("name", ""),
        description=element.get("description", ""),
        state_type=element.get("type"),
        module=element.get("module"),
        class_name=element.get("class"),
        package_name=element.get("package"),
        file_name=element.get("file_name"),
    )
    state.parameters.extend(_parse_parameters(element.findall("Param")))
    state.parameter_mappings.update(_parse_parameter_remaps(element))
    state.remappings.update(_parse_remaps(element))
    state.keys.extend(_parse_keys(element.findall("Key")))
    return state


def _parse_text_block(element: ET.Element) -> TextBlock:
    """Deserialize one free-form text block."""
    return TextBlock(
        x=_parse_float(element.get("x")) or 0.0,
        y=_parse_float(element.get("y")) or 0.0,
        content=_decode_text_content(element.get("content", "")),
    )


def _parse_transition(element: ET.Element) -> Transition:
    return Transition(
        source_outcome=element.get("from", ""),
        target=element.get("to", ""),
    )


def _parse_parameters(elements: Iterable[ET.Element]) -> list[Parameter]:
    parameters: list[Parameter] = []
    for element in elements:
        parameters.append(
            Parameter(
                name=element.get("name", ""),
                description=element.get("description", ""),
                default_type=element.get("default_type", ""),
                default_value=element.get("default_value"),
            )
        )
    return parameters


def _parse_parameter_remaps(element: ET.Element) -> dict[str, str]:
    parameter_mappings: dict[str, str] = {}
    for remap in element.findall("ParamRemap"):
        old = remap.get("old", "")
        new = remap.get("new", "")
        if old and new:
            parameter_mappings[old] = new
    return parameter_mappings


def _parse_keys(elements: Iterable[ET.Element]) -> list[Key]:
    keys: list[Key] = []
    for element in elements:
        keys.append(
            Key(
                name=element.get("name", ""),
                key_type=element.get("type", "in"),
                description=element.get("description", ""),
                default_type=element.get("default_type", ""),
                default_value=element.get("default_value"),
            )
        )
    return keys


def _parse_remaps(element: ET.Element) -> dict[str, str]:
    remappings: dict[str, str] = {}
    for remap in element.findall("Remap"):
        old = remap.get("old", "")
        new = remap.get("new", "")
        if old and new:
            remappings[old] = new
    return remappings


def _encode_text_content(value: str) -> str:
    """Persist multiline text safely inside an XML attribute."""
    return (
        value.replace("\\", "\\\\")
        .replace("\n", "\\n")
        .replace("\r", "\\r")
        .replace("\t", "\\t")
    )


def _decode_text_content(value: str) -> str:
    """Restore escaped multiline text saved inside an XML attribute."""
    if not value:
        return ""

    result: list[str] = []
    index = 0
    while index < len(value):
        char = value[index]
        if char != "\\" or index + 1 >= len(value):
            result.append(char)
            index += 1
            continue

        next_char = value[index + 1]
        if next_char == "n":
            result.append("\n")
        elif next_char == "r":
            result.append("\r")
        elif next_char == "t":
            result.append("\t")
        elif next_char == "\\":
            result.append("\\")
        else:
            result.append("\\")
            result.append(next_char)
        index += 2

    return "".join(result)


def _parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _indent(element: ET.Element, level: int = 0) -> None:
    indent = "  "
    current = "\n" + level * indent
    child_indent = "\n" + (level + 1) * indent

    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent
        for child in element:
            _indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_indent
        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = current
    elif level > 0 and (not element.tail or not element.tail.strip()):
        element.tail = current
