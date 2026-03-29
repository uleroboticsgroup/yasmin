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

from typing import Any, Optional


def container_states(container: Any) -> dict[str, Any]:
    if container is None:
        return {}

    get_states_cpp = getattr(container, "_get_states_cpp", None)
    if callable(get_states_cpp):
        try:
            states = get_states_cpp()
            if hasattr(states, "items"):
                return {str(name): state for name, state in states.items()}
        except Exception:
            return {}

    get_states = getattr(container, "get_states", None)
    if not callable(get_states):
        return {}

    try:
        states = get_states()
    except Exception:
        return {}

    if not hasattr(states, "items"):
        return {}

    normalized: dict[str, Any] = {}
    for name, value in states.items():
        child_state = value
        if isinstance(value, dict):
            child_state = value.get("state", value)
        elif hasattr(value, "get"):
            try:
                child_state = value.get("state", value)
            except Exception:
                child_state = value
        normalized[str(name)] = child_state

    return normalized


def child_state(container: Any, state_name: str) -> Optional[Any]:
    return container_states(container).get(str(state_name))


def is_container_object(state: Any) -> bool:
    if state is None:
        return False
    return callable(getattr(state, "_get_states_cpp", None)) or callable(
        getattr(state, "get_states", None)
    )


def resolve_container(root_container: Any, path: tuple[str, ...]) -> Optional[Any]:
    container = root_container
    if container is None:
        return None

    for state_name in path:
        container = child_state(container, state_name)
        if container is None:
            return None

    return container


def get_container_entry_state_name(container: Any) -> Optional[str]:
    if container is None:
        return None

    get_current_state = getattr(container, "get_current_state", None)
    if callable(get_current_state):
        try:
            current_state = get_current_state()
            if current_state:
                return str(current_state)
        except Exception:
            pass

    get_start_state = getattr(container, "get_start_state", None)
    if callable(get_start_state):
        try:
            start_state = get_start_state()
            if start_state:
                return str(start_state)
        except Exception:
            pass

    start_state = getattr(container, "start_state", None)
    if start_state:
        return str(start_state)

    return None


def expand_to_deepest_known_path(
    root_container: Any,
    base_path: tuple[str, ...],
) -> tuple[str, ...]:
    path = tuple(base_path)
    container = resolve_container(root_container, path)
    visited: set[int] = set()

    while is_container_object(container) and id(container) not in visited:
        visited.add(id(container))
        child_name = get_container_entry_state_name(container)
        if not child_name:
            break
        path = path + (child_name,)
        container = child_state(container, child_name)

    return path
