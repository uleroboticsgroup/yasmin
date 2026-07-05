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

from typing import Any, Dict, Tuple, Union, List

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

ParameterDict = Dict[str, Any]
ContainerModel = Union[StateMachine, Concurrence]


def parameters_to_dicts(parameters: List[Parameter]) -> List[ParameterDict]:
    """Convert declared parameters into editor-table dictionaries."""

    return [
        {
            "name": parameter.name,
            "description": parameter.description,
            "default_type": parameter.default_type,
            "default_value": parameter.default_value,
            "has_default": parameter.has_default,
        }
        for parameter in parameters
    ]


def dicts_to_parameters(parameters: List[ParameterDict]) -> List[Parameter]:
    """Normalize editor-table dictionaries back into parameter models."""

    normalized: List[Parameter] = []
    for item in parameters:
        name = normalize_parameter_name(item.get("name"))
        if not name:
            continue
        normalized.append(
            Parameter(
                name=name,
                description=normalize_parameter_name(item.get("description")),
                default_type=normalize_parameter_name(item.get("default_type")),
                default_value=item.get("default_value"),
            )
        )
    return normalized


def get_parameter_overwrites_for_child(
    container_model: ContainerModel,
    child_model: State,
) -> List[ParameterDict]:
    """Return child overwrite rows enriched with declared parent metadata."""

    declared_by_name = {
        parameter.name: parameter for parameter in container_model.parameters
    }
    overwrites: List[ParameterDict] = []
    for child_parameter, parent_parameter in child_model.parameter_mappings.items():
        declared = declared_by_name.get(parent_parameter)
        overwrites.append(
            {
                "name": parent_parameter,
                "child_parameter": child_parameter,
                "description": getattr(declared, "description", ""),
                "default_type": getattr(declared, "default_type", ""),
                "default_value": getattr(declared, "default_value", ""),
            }
        )
    return overwrites


def apply_parameter_overwrites(
    container_model: ContainerModel,
    child_model: State,
    overwrites: List[ParameterDict],
) -> None:
    """Apply one child overwrite table back into the container and child models.

    The container keeps only declarations that remain referenced by at least one
    child after the update. Declaration order is preserved for existing entries,
    while newly introduced names are appended.
    """

    child_model.parameter_mappings.clear()
    for child_parameter, parent_parameter in iter_valid_overwrite_pairs(overwrites):
        child_model.parameter_mappings[child_parameter] = parent_parameter

    declarations_by_name = {
        parameter.name: parameter for parameter in container_model.parameters
    }
    declaration_order = [parameter.name for parameter in container_model.parameters]

    for declaration in iter_declared_parameters(overwrites):
        declarations_by_name[declaration.name] = declaration
        if declaration.name not in declaration_order:
            declaration_order.append(declaration.name)

    used_names = {
        parent_name
        for state in container_model.states.values()
        for parent_name in state.parameter_mappings.values()
    }
    container_model.parameters = [
        declarations_by_name[name]
        for name in declaration_order
        if name in declarations_by_name and name in used_names
    ]


def normalize_parameter_name(value: Any) -> str:
    """Return a stripped string representation for user-entered names."""

    return str(value or "").strip()


def iter_valid_overwrite_pairs(
    overwrites: List[ParameterDict],
) -> List[Tuple[str, str]]:
    """Return valid child-to-parent overwrite pairs from editor rows."""

    pairs: List[Tuple[str, str]] = []
    for item in overwrites:
        child_parameter = normalize_parameter_name(item.get("child_parameter"))
        parent_parameter = normalize_parameter_name(item.get("name"))
        if child_parameter and parent_parameter:
            pairs.append((child_parameter, parent_parameter))
    return pairs


def iter_declared_parameters(overwrites: List[ParameterDict]) -> List[Parameter]:
    """Return normalized parent parameter declarations from overwrite rows."""

    declarations: List[Parameter] = []
    for item in overwrites:
        name = normalize_parameter_name(item.get("name"))
        if not name:
            continue
        declarations.append(
            Parameter(
                name=name,
                description=normalize_parameter_name(item.get("description")),
                default_type=normalize_parameter_name(item.get("default_type")),
                default_value=item.get("default_value"),
            )
        )
    return declarations
