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
"""Adapters for turning dialog tuples into canonical state-creation kwargs.

The editor still receives tuple-shaped results from several legacy dialogs.
Keeping the mapping in one place makes the create-state flow easier to read and
unit test, and avoids repeating container defaults in multiple call sites.
"""

from __future__ import annotations

from typing import Any, Dict, Tuple

PluginStateDialogResult = Tuple[
    str,
    Any,
    list[str],
    dict[str, str],
    str,
    list[dict[str, Any]],
    list[dict[str, Any]],
]
StateMachineDialogResult = Tuple[
    str,
    list[str],
    str | None,
    dict[str, str],
    str,
    list[dict[str, Any]],
]
ConcurrenceDialogResult = Tuple[
    str,
    list[str],
    str | None,
    dict[str, str],
    str,
    list[dict[str, Any]],
]


def _build_container_kwargs(
    *,
    name: str,
    outcomes: list[str],
    remappings: dict[str, str],
    description: str,
    defaults: list[dict[str, Any]],
    is_state_machine: bool,
    start_state: str | None,
    default_outcome: str | None,
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
