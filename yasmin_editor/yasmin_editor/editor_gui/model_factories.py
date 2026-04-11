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
"""Helpers for resolving plugin metadata and constructing editor models.

The main editor window needs to translate plugin-manager entries into concrete
state models, but that mapping logic does not need to live inside a Qt mixin.
Keeping it here makes plugin resolution and model creation easy to test without
booting the full editor.
"""

from __future__ import annotations

from typing import Any

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

PluginManager = Any
PluginInfoLike = Any


def resolve_plugin_info_for_model(
    plugin_manager: PluginManager, model: State
) -> PluginInfoLike:
    """Resolve the plugin-manager entry that best matches one state model."""

    if model.state_type == "py":
        plugin = next(
            (
                item
                for item in plugin_manager.python_plugins
                if item.module == model.module and item.class_name == model.class_name
            ),
            None,
        )
    elif model.state_type == "cpp":
        plugin = next(
            (
                item
                for item in plugin_manager.cpp_plugins
                if item.class_name == model.class_name
            ),
            None,
        )
    else:
        plugin = next(
            (
                item
                for item in plugin_manager.xml_files
                if item.file_name == model.file_name
                and (not model.package_name or item.package_name == model.package_name)
            ),
            None,
        )
        if plugin is None and model.file_name:
            plugin = next(
                (
                    item
                    for item in plugin_manager.xml_files
                    if item.file_name == model.file_name
                ),
                None,
            )

    if plugin is None:
        raise ValueError(f"Unable to resolve plugin for state '{model.name}'")
    return plugin


def create_leaf_model(
    name: str,
    plugin_info: PluginInfoLike,
    *,
    description: str = "",
    remappings: dict[str, str] | None = None,
    parameter_mappings: dict[str, str] | None = None,
    outcomes: list[str] | None = None,
) -> State:
    """Create one leaf-state model from plugin metadata."""

    state_type = plugin_type_to_state_type(getattr(plugin_info, "plugin_type", None))
    model = State(
        name=name,
        description=description or "",
        remappings=dict(remappings or {}),
        parameter_mappings=dict(parameter_mappings or {}),
        state_type=state_type,
        module=getattr(plugin_info, "module", None),
        class_name=getattr(plugin_info, "class_name", None),
        package_name=getattr(plugin_info, "package_name", None),
        file_name=getattr(plugin_info, "file_name", None),
    )
    for outcome_name in iter_leaf_outcome_names(plugin_info, outcomes):
        model.add_outcome(Outcome(name=outcome_name))
    return model


def create_container_model(
    name: str,
    *,
    is_concurrence: bool,
    outcomes: list[str] | None = None,
    remappings: dict[str, str] | None = None,
    start_state: str | None = None,
    default_outcome: str | None = None,
    description: str = "",
    parameter_mappings: dict[str, str] | None = None,
) -> StateMachine | Concurrence:
    """Create one state-machine or concurrence model for the editor."""

    common_kwargs = dict(
        name=name,
        description=description or "",
        remappings=dict(remappings or {}),
        parameter_mappings=dict(parameter_mappings or {}),
    )
    if is_concurrence:
        model: StateMachine | Concurrence = Concurrence(
            default_outcome=default_outcome,
            **common_kwargs,
        )
    else:
        model = StateMachine(
            start_state=start_state,
            **common_kwargs,
        )
    for outcome_name in outcomes or []:
        model.add_outcome(Outcome(name=outcome_name))
    return model


def plugin_type_to_state_type(plugin_type: str | None) -> str | None:
    """Map plugin-manager plugin types onto serialized model state types."""

    if plugin_type is None:
        return None
    return {"python": "py", "cpp": "cpp", "xml": "xml"}.get(plugin_type, plugin_type)


def iter_leaf_outcome_names(
    plugin_info: PluginInfoLike,
    overrides: list[str] | None = None,
) -> list[str]:
    """Return the outcome names that should be attached to a leaf model."""

    return list(overrides or getattr(plugin_info, "outcomes", []) or [])
