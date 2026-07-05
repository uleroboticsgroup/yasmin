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

"""Helpers for traversing live YASMIN runtime container objects.

The runtime interacts with Python and C++ backed state containers. These
helpers normalize that mixed API surface so the runtime code can reason about
paths and nested containers without scattering compatibility checks.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Set, Tuple


def container_states(container: Any) -> Dict[str, Any]:
    """Return a normalized mapping of child state names to runtime objects."""
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

    normalized: Dict[str, Any] = {}
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
    """Return one named child state from a container if it exists."""
    return container_states(container).get(str(state_name))


def is_concurrence_object(state: Any) -> bool:
    """Return whether the runtime object behaves like a concurrence."""
    if state is None:
        return False

    try:
        import yasmin

        concurrence_type = getattr(yasmin, "Concurrence", None)
        if concurrence_type is not None and isinstance(state, concurrence_type):
            return True
    except Exception:
        pass

    state_type = type(state)
    return state_type.__name__ == "Concurrence"


def is_container_object(state: Any) -> bool:
    """Return whether the runtime object exposes container child-state access."""
    if state is None:
        return False
    return callable(getattr(state, "_get_states_cpp", None)) or callable(
        getattr(state, "get_states", None)
    )


def resolve_container(root_container: Any, path: Tuple[str, ...]) -> Optional[Any]:
    """Resolve a nested container path from the runtime root object."""
    container = root_container
    if container is None:
        return None

    for state_name in path:
        container = child_state(container, state_name)
        if container is None:
            return None

    return container


def get_container_entry_state_name(container: Any) -> Optional[str]:
    """Return the entry state name used when stepping into a container."""
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
    base_path: Tuple[str, ...],
) -> Tuple[str, ...]:
    """Follow container entry states until the deepest known active path is reached."""
    path = tuple(base_path)
    container = resolve_container(root_container, path)
    visited: Set[int] = set()

    while is_container_object(container) and id(container) not in visited:
        if is_concurrence_object(container):
            break

        visited.add(id(container))
        child_name = get_container_entry_state_name(container)
        if not child_name:
            break
        path = path + (child_name,)
        container = child_state(container, child_name)

    return path
