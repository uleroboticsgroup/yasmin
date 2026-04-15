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
"""Helpers for editor-facing parameter declarations and overwrite mappings.

These helpers keep the parameter editing rules out of the Qt mixins so the
conversion and update logic can be tested directly. The editor still owns when
those helpers are called, but this module owns how parameter declarations and
child overwrite mappings are normalized.
"""

from __future__ import annotations

from typing import Any

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

ParameterDict = dict[str, Any]
ContainerModel = StateMachine | Concurrence


def parameters_to_dicts(parameters: list[Parameter]) -> list[ParameterDict]:
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


def dicts_to_parameters(parameters: list[ParameterDict]) -> list[Parameter]:
    """Normalize editor-table dictionaries back into parameter models."""

    normalized: list[Parameter] = []
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
) -> list[ParameterDict]:
    """Return child overwrite rows enriched with declared parent metadata."""

    declared_by_name = {
        parameter.name: parameter for parameter in container_model.parameters
    }
    overwrites: list[ParameterDict] = []
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
    overwrites: list[ParameterDict],
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
    overwrites: list[ParameterDict],
) -> list[tuple[str, str]]:
    """Return valid child-to-parent overwrite pairs from editor rows."""

    pairs: list[tuple[str, str]] = []
    for item in overwrites:
        child_parameter = normalize_parameter_name(item.get("child_parameter"))
        parent_parameter = normalize_parameter_name(item.get("name"))
        if child_parameter and parent_parameter:
            pairs.append((child_parameter, parent_parameter))
    return pairs


def iter_declared_parameters(overwrites: list[ParameterDict]) -> list[Parameter]:
    """Return normalized parent parameter declarations from overwrite rows."""

    declarations: list[Parameter] = []
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
