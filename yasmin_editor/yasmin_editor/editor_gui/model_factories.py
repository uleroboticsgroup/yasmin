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

from typing import Any, Dict, List, Union

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
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
        return None
    return plugin


def create_leaf_model(
    name: str,
    plugin_info: PluginInfoLike,
    *,
    description: str = "",
    remappings: Union[Dict[str, str], None] = None,
    parameter_mappings: Union[Dict[str, str], None] = None,
    outcomes: Union[List[str], None] = None,
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
    is_concurrence: bool = False,
    is_orthogonal: bool = False,
    outcomes: Union[List[str], None] = None,
    remappings: Union[Dict[str, str], None] = None,
    start_state: Union[str, None] = None,
    default_outcome: Union[str, None] = None,
    description: str = "",
    parameter_mappings: Union[Dict[str, str], None] = None,
) -> Union[StateMachine, Concurrence, OrthogonalState]:
    """Create one state-machine, concurrence, or orthogonal model for the editor."""

    common_kwargs = dict(
        name=name,
        description=description or "",
        remappings=dict(remappings or {}),
        parameter_mappings=dict(parameter_mappings or {}),
    )
    if is_orthogonal:
        model: Union[StateMachine, Concurrence, OrthogonalState] = OrthogonalState(
            default_outcome=default_outcome,
            **common_kwargs,
        )
    elif is_concurrence:
        model = Concurrence(
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


def plugin_type_to_state_type(plugin_type: Union[str, None]) -> Union[str, None]:
    """Map plugin-manager plugin types onto serialized model state types."""

    if plugin_type is None:
        return None
    return {"python": "py", "cpp": "cpp", "xml": "xml"}.get(plugin_type, plugin_type)


def iter_leaf_outcome_names(
    plugin_info: PluginInfoLike,
    overrides: Union[List[str], None] = None,
) -> List[str]:
    """Return the outcome names that should be attached to a leaf model."""

    return list(overrides or getattr(plugin_info, "outcomes", []) or [])
