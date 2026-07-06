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

from pathlib import Path
from typing import Callable, Dict, Iterable, List, Set, Tuple, Union
from xml.etree import ElementTree as ET

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.container_state import iter_outcome_rule_values
from yasmin_editor.model.join_state import JoinState
from yasmin_editor.model.key import Key
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition


def model_to_xml(model: StateMachine, file_path: Union[str, Path] = None) -> str:
    """Serialize a state machine model to XML."""

    root = _state_machine_to_element(model, parent=None)
    if hasattr(ET, "indent"):
        ET.indent(root)
    xml_text = ET.tostring(root, encoding="utf-8", xml_declaration=True).decode("utf-8")

    if file_path is not None:
        Path(file_path).write_text(xml_text, encoding="utf-8")

    return xml_text


def model_from_xml(xml_input: Union[str, Path]) -> StateMachine:
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


def _container_to_element(
    tag: str,
    model: Union[StateMachine, Concurrence, OrthogonalState],
    parent: Union[StateMachine, Concurrence, OrthogonalState, None],
) -> ET.Element:
    element = ET.Element(tag)
    element.set("name", model.name)

    if isinstance(model, StateMachine):
        if model.outcomes:
            element.set("outcomes", " ".join(o.name for o in model.outcomes))
        if model.start_state:
            element.set("start_state", model.start_state)
    else:
        outcome_names = {o.name for o in model.outcomes}
        if model.default_outcome:
            outcome_names.add(model.default_outcome)
        if outcome_names:
            element.set("outcomes", " ".join(sorted(outcome_names)))
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

    if isinstance(model, OrthogonalState):
        for state in model.states.values():
            if isinstance(state, StateMachine):
                region_elem = _region_to_element(state, model)
            else:
                region_elem = ET.Element("Region")
                region_elem.set("name", state.name)
                _append_owner_transitions(region_elem, model, state.name)
            element.append(region_elem)
    else:
        for state in model.states.values():
            element.append(_state_to_element(state, model))

    for outcome in model.outcomes:
        for outcome_element in _final_outcome_elements(outcome, model):
            element.append(outcome_element)

    if isinstance(model, (Concurrence, OrthogonalState)):
        _append_outcome_map(element, model)

    if isinstance(model, StateMachine):
        if parent is None:
            _append_container_level_transitions(element, model)
        else:
            _append_owner_transitions(element, parent, model.name)
    elif parent is not None and isinstance(parent, StateMachine):
        _append_owner_transitions(element, parent, model.name)

    return element


def _state_machine_to_element(
    model: StateMachine,
    parent: Union[StateMachine, Concurrence, OrthogonalState, None],
) -> ET.Element:
    return _container_to_element("StateMachine", model, parent)


def _concurrence_to_element(
    model: Concurrence,
    parent: Union[StateMachine, Concurrence, OrthogonalState, None],
) -> ET.Element:
    return _container_to_element("Concurrence", model, parent)


def _orthogonal_state_to_element(
    model: OrthogonalState,
    parent: Union[StateMachine, Concurrence, OrthogonalState, None],
) -> ET.Element:
    return _container_to_element("OrthogonalState", model, parent)


def _region_to_element(
    region: StateMachine,
    parent: OrthogonalState,
) -> ET.Element:
    element = ET.Element("Region")
    element.set("name", region.name)

    if region.outcomes:
        element.set("outcomes", " ".join(o.name for o in region.outcomes))
    if region.start_state:
        element.set("start_state", region.start_state)
    if region.description:
        element.set("description", region.description)

    position = parent.layout.get_state_position(region.name)
    if position is not None:
        element.set("x", f"{position.x:.2f}")
        element.set("y", f"{position.y:.2f}")

    for parameter in region.parameters:
        element.append(_parameter_to_element(parameter))

    _append_parameter_remaps(element, region.parameter_mappings)
    _append_remaps(element, region.remappings)

    for text_block in region.text_blocks:
        element.append(_text_block_to_element(text_block))

    for state in region.states.values():
        element.append(_state_to_element(state, region))

    for outcome in region.outcomes:
        for outcome_element in _final_outcome_elements(outcome, region):
            element.append(outcome_element)

    return element


def _state_to_element(
    state: State, parent: Union[StateMachine, Concurrence, OrthogonalState, None]
) -> ET.Element:
    if isinstance(state, StateMachine):
        return _state_machine_to_element(state, parent=parent)
    if isinstance(state, Concurrence):
        return _concurrence_to_element(state, parent=parent)
    if isinstance(state, OrthogonalState):
        return _orthogonal_state_to_element(state, parent=parent)

    if isinstance(state, JoinState):
        element = ET.Element("JoinState")
        element.set("name", state.name)
        if state.sync_id:
            element.set("sync_id", state.sync_id)
        element.set("outcome", state.join_outcome)
        if state.description:
            element.set("description", state.description)
        position = parent.layout.get_state_position(state.name)
        if position is not None:
            element.set("x", f"{position.x:.2f}")
            element.set("y", f"{position.y:.2f}")
        _append_parameter_remaps(element, state.parameter_mappings)
        _append_remaps(element, state.remappings)
        _append_owner_transitions(element, parent, state.name)
        return element

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
    element.set("content", _normalize_newlines(text_block.content))
    return element


def _final_outcome_elements(
    outcome: Outcome,
    parent: Union[StateMachine, Concurrence, OrthogonalState],
) -> List[ET.Element]:
    """Serialize one logical outcome without mutating the in-memory layout."""

    serializable_placements = _serializable_outcome_placements(parent, outcome.name)

    elements: List[ET.Element] = []
    for index, placement in enumerate(serializable_placements):
        element = ET.Element("FinalOutcome")
        element.set("name", outcome.name)

        if outcome.description:
            element.set("description", outcome.description)

        if placement is not None:
            instance_id, x, y = placement
            if instance_id:
                element.set("instance_id", instance_id)
            element.set("x", f"{x:.2f}")
            element.set("y", f"{y:.2f}")

        if isinstance(parent, StateMachine) and index == 0:
            for transition in parent.transitions.get(outcome.name, []):
                element.append(_transition_to_element(transition))

        elements.append(element)
    return elements


def _serializable_outcome_placements(
    parent: Union[StateMachine, Concurrence, OrthogonalState],
    outcome_name: str,
) -> List[Union[Tuple[Union[str, None], float, float], None]]:
    """Return outcome placements for XML serialization without changing layout state.

    Legacy layouts may still store only the primary ``outcome_positions`` entry without
    an explicit alias placement. In that case the serializer emits one placement without
    an ``instance_id`` instead of materialising a random alias inside the live model.
    """

    placements = parent.layout.get_outcome_placements(outcome_name)
    if placements:
        return [
            (
                placement.instance_id,
                placement.position.x,
                placement.position.y,
            )
            for placement in placements
        ]

    primary_position = parent.layout.get_outcome_position(outcome_name)
    if primary_position is None:
        return [None]

    return [(None, primary_position.x, primary_position.y)]


def _transition_to_element(transition: Transition) -> ET.Element:
    element = ET.Element("Transition")
    element.set("from", transition.source_outcome)
    element.set("to", transition.target)
    if transition.target_instance_id:
        element.set("to_instance", transition.target_instance_id)
    return element


def _data_to_element(
    tag: str,
    obj: Key | Parameter,
    type_attr: Union[str, None] = None,
) -> ET.Element:
    element = ET.Element(tag)
    element.set("name", obj.name)

    if type_attr is not None:
        element.set("type", type_attr)

    if obj.description:
        element.set("description", obj.description)
    if obj.has_default:
        element.set("default_type", obj.default_type)
        element.set(
            "default_value",
            "" if obj.default_value is None else str(obj.default_value),
        )

    return element


def _key_to_element(key: Key) -> ET.Element:
    return _data_to_element("Key", key, type_attr=key.key_type)


def _parameter_to_element(parameter: Parameter) -> ET.Element:
    return _data_to_element("Param", parameter)


def _append_remap_elements(
    element: ET.Element,
    mappings: Dict[str, str],
    tag: str,
) -> None:
    for old, new in mappings.items():
        if not old or not new:
            continue
        remap = ET.SubElement(element, tag)
        remap.set("old", old)
        remap.set("new", new)


def _append_parameter_remaps(
    element: ET.Element,
    parameter_mappings: Dict[str, str],
) -> None:
    _append_remap_elements(element, parameter_mappings, "ParamRemap")


def _append_remaps(element: ET.Element, remappings: Dict[str, str]) -> None:
    _append_remap_elements(element, remappings, "Remap")


def _append_owner_transitions(
    element: ET.Element,
    parent: Union[StateMachine, Concurrence, OrthogonalState],
    owner_name: str,
) -> None:
    if isinstance(parent, (Concurrence, OrthogonalState)):
        return
    for transition in parent.transitions.get(owner_name, []):
        element.append(_transition_to_element(transition))


def _append_container_level_transitions(
    element: ET.Element,
    model: StateMachine,
) -> None:
    child_names = set(model.states)
    child_names.update(outcome.name for outcome in model.outcomes)

    for owner_name, transition_list in model.transitions.items():
        if owner_name in child_names:
            continue
        for transition in transition_list:
            element.append(_transition_to_element(transition))


def _append_outcome_map(
    element: ET.Element,
    model: Union[Concurrence, OrthogonalState],
) -> None:
    for outcome_name, requirements in model.outcome_map.items():
        outcome_map_elem = ET.SubElement(element, "OutcomeMap")
        outcome_map_elem.set("outcome", outcome_name)

        for state_name, state_outcomes in requirements.items():
            for state_outcome in iter_outcome_rule_values(state_outcomes):
                item_elem = ET.SubElement(outcome_map_elem, "Item")
                item_elem.set("state", state_name)
                item_elem.set("outcome", state_outcome)


def _parse_container_element(
    element: ET.Element,
    model_class: type,
    content_parser: Callable,
    **model_kwargs,
):
    model = model_class(
        name=element.get("name", ""),
        description=element.get("description", ""),
        **model_kwargs,
    )

    model.parameters.extend(_parse_parameters(element.findall("Param")))
    model.keys.extend(_parse_keys(element.findall("Key")))
    model.parameter_mappings.update(_parse_parameter_remaps(element))
    model.remappings.update(_parse_remaps(element))
    content_parser(model, element)
    return model


def _parse_state_machine_container(
    element: ET.Element,
    *,
    include_container_level_transitions: bool = True,
) -> StateMachine:
    def _parse_content(model, el):
        _parse_state_machine_content(
            model,
            el,
            include_container_level_transitions=include_container_level_transitions,
        )

    return _parse_container_element(
        element,
        StateMachine,
        _parse_content,
        start_state=element.get("start_state"),
    )


def _parse_concurrence_container(element: ET.Element) -> Concurrence:
    return _parse_container_element(
        element,
        Concurrence,
        _parse_container_content,
        default_outcome=element.get("default_outcome"),
    )


def _parse_orthogonal_state_container(element: ET.Element) -> OrthogonalState:
    return _parse_container_element(
        element,
        OrthogonalState,
        _parse_container_content,
        default_outcome=element.get("default_outcome"),
    )


def _parse_container_content(
    model: Concurrence | OrthogonalState,
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

        if isinstance(model, OrthogonalState):
            region = _parse_region_container(child)
            model.add_state(region)
            x = _parse_float(child.get("x"))
            y = _parse_float(child.get("y"))
            if x is not None and y is not None:
                model.layout.set_state_position(region.name, x, y)
        else:
            state = _parse_state_like(child)
            model.add_state(state)
            x = _parse_float(child.get("x"))
            y = _parse_float(child.get("y"))
            if x is not None and y is not None:
                model.layout.set_state_position(state.name, x, y)

    if model.default_outcome and not model.get_outcome(model.default_outcome):
        model.add_outcome(Outcome(name=model.default_outcome))


def _parse_region_container(element: ET.Element) -> StateMachine:
    region = StateMachine(
        name=element.get("name", ""),
        description=element.get("description", ""),
        start_state=element.get("start_state"),
    )

    outcome_names = element.get("outcomes", "").split()
    for outcome_name in outcome_names:
        region.add_outcome(Outcome(name=outcome_name))

    region.parameters.extend(_parse_parameters(element.findall("Param")))
    region.parameter_mappings.update(_parse_parameter_remaps(element))
    region.remappings.update(_parse_remaps(element))

    local_targets = _local_state_machine_targets(element)

    for child in element:
        if child.tag in {"Param", "Key", "ParamRemap", "Remap"}:
            continue

        if child.tag == "Text":
            region.add_text_block(_parse_text_block(child))
            continue

        if child.tag == "FinalOutcome":
            _merge_final_outcome(region, child)
            continue

        if child.tag == "Transition":
            continue

        state = _parse_state_like(child)
        region.add_state(state)

        x = _parse_float(child.get("x"))
        y = _parse_float(child.get("y"))
        if x is not None and y is not None:
            region.layout.set_state_position(state.name, x, y)

        for transition_elem in child.findall("Transition"):
            transition = _parse_transition(transition_elem)
            if _is_leaked_same_name_container_transition(
                region,
                state.name,
                transition,
                local_targets,
            ):
                continue
            region.add_transition(state.name, transition)

    return region


def _parse_state_machine_content(
    model: StateMachine,
    element: ET.Element,
    *,
    include_container_level_transitions: bool,
) -> None:
    outcome_names = element.get("outcomes", "").split()
    for outcome_name in outcome_names:
        model.add_outcome(Outcome(name=outcome_name))

    local_targets = _local_state_machine_targets(element)

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
            if include_container_level_transitions:
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
            transition = _parse_transition(transition_elem)
            if _is_leaked_same_name_container_transition(
                model,
                state.name,
                transition,
                local_targets,
            ):
                continue
            model.add_transition(state.name, transition)


def _local_state_machine_targets(element: ET.Element) -> Set[str]:
    targets = set(element.get("outcomes", "").split())

    for child in element:
        if child.tag in {
            "State",
            "StateMachine",
            "Concurrence",
            "OrthogonalState",
            "Region",
            "JoinState",
        }:
            name = child.get("name", "")
            if name:
                targets.add(name)
        elif child.tag == "FinalOutcome":
            name = child.get("name", "")
            if name:
                targets.add(name)

    return targets


def _is_leaked_same_name_container_transition(
    model: StateMachine,
    state_name: str,
    transition: Transition,
    local_targets: Set[str],
) -> bool:
    return (
        state_name == model.name
        and transition.target not in local_targets
        and bool(transition.target)
    )


def _merge_final_outcome(
    model: Union[StateMachine, Concurrence, OrthogonalState],
    element: ET.Element,
) -> None:
    """Merge one serialized final outcome into the in-memory model.

    Legacy XML may omit ``instance_id`` for the primary outcome placement. That
    representation should remain stable across load/save roundtrips, so the
    first legacy placement is restored as a plain primary position instead of a
    generated alias instance.
    """

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
        instance_id = element.get("instance_id")
        if instance_id:
            model.layout.set_outcome_position(
                outcome.name,
                x,
                y,
                instance_id=instance_id,
            )
        elif not model.layout.get_outcome_placements(outcome.name) and (
            outcome.name not in model.layout.outcome_positions
        ):
            model.layout.set_primary_outcome_position(
                outcome.name,
                x,
                y,
            )
        else:
            model.layout.materialize_primary_outcome_position(outcome.name)
            model.layout.create_outcome_alias(
                outcome.name,
                x,
                y,
            )

    if isinstance(model, StateMachine):
        for transition_elem in element.findall("Transition"):
            model.add_transition(outcome.name, _parse_transition(transition_elem))


def _parse_outcome_map(
    model: Union[Concurrence, OrthogonalState],
    element: ET.Element,
) -> None:
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

    if element.tag == "OrthogonalState":
        return _parse_orthogonal_state_container(element)

    if element.tag == "StateMachine" and not element.get("file_name"):
        return _parse_state_machine_container(
            element,
            include_container_level_transitions=False,
        )

    if element.tag == "JoinState":
        state = JoinState(
            name=element.get("name", ""),
            description=element.get("description", ""),
            sync_id=element.get("sync_id", ""),
            join_outcome=element.get("outcome", "joined"),
        )
        state.parameter_mappings.update(_parse_parameter_remaps(element))
        state.remappings.update(_parse_remaps(element))
        return state

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
        content=_normalize_newlines(element.get("content", "")),
    )


def _parse_transition(element: ET.Element) -> Transition:
    return Transition(
        source_outcome=element.get("from", ""),
        target=element.get("to", ""),
        target_instance_id=element.get("to_instance", ""),
    )


def _parse_typed_elements(
    elements: Iterable[ET.Element],
    cls: type,
    field_fn: Union[Callable[[ET.Element], dict], None] = None,
) -> list:
    result = []
    for element in elements:
        kwargs = {
            "name": element.get("name", ""),
            "description": element.get("description", ""),
            "default_type": element.get("default_type", ""),
            "default_value": element.get("default_value"),
        }
        if field_fn:
            kwargs.update(field_fn(element))
        result.append(cls(**kwargs))
    return result


def _parse_parameters(elements: Iterable[ET.Element]) -> List[Parameter]:
    return _parse_typed_elements(elements, Parameter)


def _parse_keys(elements: Iterable[ET.Element]) -> List[Key]:
    return _parse_typed_elements(
        elements,
        Key,
        field_fn=lambda e: {"key_type": e.get("type", "in")},
    )


def _parse_remap_dict(element: ET.Element, tag: str) -> Dict[str, str]:
    mappings: Dict[str, str] = {}
    for remap in element.findall(tag):
        old = remap.get("old", "")
        new = remap.get("new", "")
        if old and new:
            mappings[old] = new
    return mappings


def _parse_parameter_remaps(element: ET.Element) -> Dict[str, str]:
    return _parse_remap_dict(element, "ParamRemap")


def _parse_remaps(element: ET.Element) -> Dict[str, str]:
    return _parse_remap_dict(element, "Remap")


def _normalize_newlines(value: str) -> str:
    return value.replace("\r\n", "\n").replace("\r", "\n")


def _parse_float(value: Union[str, None]) -> Union[float, None]:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None
