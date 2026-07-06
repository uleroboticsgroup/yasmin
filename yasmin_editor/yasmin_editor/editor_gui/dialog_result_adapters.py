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

from typing import Any, Dict, Tuple, List, Union

PluginStateDialogResult = Tuple[
    str,
    Any,
    List[str],
    Dict[str, str],
    str,
    List[Dict[str, Any]],
    List[Dict[str, Any]],
]
StateMachineDialogResult = Tuple[
    str,
    List[str],
    Union[str, None],
    Dict[str, str],
    str,
    List[Dict[str, Any]],
]
ConcurrenceDialogResult = Tuple[
    str,
    List[str],
    Union[str, None],
    Dict[str, str],
    str,
    List[Dict[str, Any]],
]
OrthogonalStateDialogResult = Tuple[
    str,
    List[str],
    Union[str, None],
    Dict[str, str],
    str,
    List[Dict[str, Any]],
]
JoinStateDialogResult = Tuple[
    str,
    str,
    str,
    str,
]


def _build_container_kwargs(
    *,
    name: str,
    outcomes: List[str],
    remappings: Dict[str, str],
    description: str,
    defaults: List[Dict[str, Any]],
    is_state_machine: bool,
    start_state: Union[str, None],
    default_outcome: Union[str, None],
) -> Dict[str, Any]:
    """Build the shared kwargs dictionary for container-like editor nodes."""

    return {
        "name": name,
        "plugin_info": None,
        "is_state_machine": is_state_machine,
        "is_concurrence": not is_state_machine,
        "outcomes": outcomes,
        "remappings": remappings,
        "start_state": start_state,
        "default_outcome": default_outcome,
        "description": description,
        "defaults": defaults,
    }


def build_plugin_state_kwargs(result: PluginStateDialogResult) -> Dict[str, Any]:
    """Adapt a plugin-state dialog result to ``create_state_node`` kwargs."""

    name, plugin, outcomes, remappings, description, defaults, parameter_overwrites = (
        result
    )
    return {
        "name": name,
        "plugin_info": plugin,
        "outcomes": outcomes,
        "remappings": remappings,
        "description": description,
        "defaults": defaults,
        "parameter_overwrites": parameter_overwrites,
    }


def build_state_machine_kwargs(result: StateMachineDialogResult) -> Dict[str, Any]:
    """Adapt a state-machine dialog result to ``create_state_node`` kwargs."""

    name, outcomes, start_state, remappings, description, defaults = result
    return _build_container_kwargs(
        name=name,
        outcomes=outcomes,
        remappings=remappings,
        description=description,
        defaults=defaults,
        is_state_machine=True,
        start_state=start_state,
        default_outcome=None,
    )


def build_concurrence_kwargs(result: ConcurrenceDialogResult) -> Dict[str, Any]:
    """Adapt a concurrence dialog result to ``create_state_node`` kwargs."""

    name, outcomes, default_outcome, remappings, description, defaults = result
    return _build_container_kwargs(
        name=name,
        outcomes=outcomes,
        remappings=remappings,
        description=description,
        defaults=defaults,
        is_state_machine=False,
        start_state=None,
        default_outcome=default_outcome,
    )


def build_orthogonal_state_kwargs(result: OrthogonalStateDialogResult) -> Dict[str, Any]:
    """Adapt an orthogonal state dialog result to ``create_state_node`` kwargs."""

    name, outcomes, default_outcome, remappings, description, defaults = result
    return {
        "name": name,
        "plugin_info": None,
        "is_orthogonal": True,
        "outcomes": outcomes,
        "remappings": remappings,
        "start_state": None,
        "default_outcome": default_outcome,
        "description": description,
        "defaults": defaults,
    }


def build_join_state_kwargs(result: JoinStateDialogResult) -> Dict[str, Any]:
    """Adapt a join-state dialog result to ``create_state_node`` kwargs."""

    name, sync_id, outcome, description = result
    return {
        "name": name,
        "plugin_info": None,
        "is_join_state": True,
        "sync_id": sync_id,
        "join_outcome": outcome,
        "description": description,
    }
