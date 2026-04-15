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
"""Qt-free helpers for blackboard key filtering, remapping, and aggregation.

These helpers keep the editor blackboard mixin focused on widget orchestration.
The rules in this module are intentionally pure so they can be covered directly
with unit tests without requiring a running Qt application.
"""

from __future__ import annotations

from typing import Callable, Dict, Iterable, List, Optional, Tuple

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

ResolvePluginInfo = Callable[[State], object]


def should_hide_blackboard_key(
    *,
    item_text: str,
    key_name: str,
    filter_text: str,
    show_hidden: bool,
) -> bool:
    """Return whether one blackboard-list entry should be hidden.

    Hidden keys are names starting with ``.``. They remain available in the
    data model but are optionally filtered out of the sidebar view.
    """

    is_hidden_key = str(key_name or "").startswith(".")
    normalized_filter = str(filter_text or "").lower()
    matches_filter = normalized_filter in str(item_text or "").lower()
    return (not show_hidden and is_hidden_key) or not matches_filter


def format_blackboard_key_label(key_data: Dict[str, str]) -> str:
    """Return the sidebar label for one blackboard key entry."""

    label = f"{key_data.get('name', '')} ({key_data.get('key_type', 'in')})"

    default_type = str(key_data.get("default_type", "")).strip()
    if default_type:
        default_value = str(key_data.get("default_value", ""))
        label += f" [default: {default_value}, type: {default_type}]"

    return label


def dicts_to_keys(keys: List[Dict[str, str]]) -> List[Key]:
    """Normalize editor-table rows into blackboard-key models."""

    result: List[Key] = []
    for key_data in keys:
        key_name = str(key_data.get("name", "") or "").strip()
        if not key_name:
            continue
        result.append(
            Key(
                name=key_name,
                key_type=str(key_data.get("key_type", "in") or "in").strip(),
                description=str(key_data.get("description", "") or "").strip(),
                default_type=str(key_data.get("default_type", "") or "").strip(),
                default_value=str(key_data.get("default_value", "") or ""),
            )
        )
    return result


def keys_to_dicts(keys: Iterable[Key]) -> List[Dict[str, str]]:
    """Convert blackboard-key models back into editor-table rows."""

    result: List[Dict[str, str]] = []
    for key in keys:
        result.append(
            {
                "name": key.name,
                "key_type": key.key_type,
                "description": key.description,
                "default_type": key.default_type,
                "default_value": (
                    "" if key.default_value is None else str(key.default_value)
                ),
            }
        )
    return result


def get_effective_blackboard_key_name(
    remap_chain: Iterable[Dict[str, str]],
    key_name: str,
) -> str:
    """Resolve one key name through a chain of remapping dictionaries."""

    effective_key_name = key_name
    for remappings in remap_chain:
        effective_key_name = remappings.get(effective_key_name, effective_key_name)
    return effective_key_name


def build_container_metadata_map(
    container_model: StateMachine | Concurrence,
) -> Dict[str, Dict[str, str]]:
    """Return persistent blackboard metadata stored on one container model."""

    metadata: Dict[str, Dict[str, str]] = {}
    for key in getattr(container_model, "keys", []) or []:
        key_name = str(getattr(key, "name", "") or "").strip()
        if not key_name:
            continue
        metadata[key_name] = {
            "description": str(getattr(key, "description", "") or "").strip(),
            "key_type": str(getattr(key, "key_type", "in") or "in").strip(),
            "default_type": str(getattr(key, "default_type", "") or "").strip(),
            "default_value": (
                ""
                if getattr(key, "default_value", None) is None
                else str(getattr(key, "default_value", ""))
            ),
        }
    return metadata


def metadata_map_to_keys(metadata: Dict[str, Dict[str, str]]) -> List[Key]:
    """Convert one metadata map into sorted key models."""

    return dicts_to_keys(
        [
            {
                "name": key_name,
                "key_type": values.get("key_type", "in"),
                "description": values.get("description", ""),
                "default_type": values.get("default_type", ""),
                "default_value": values.get("default_value", ""),
            }
            for key_name, values in sorted(
                metadata.items(), key=lambda item: item[0].lower()
            )
        ]
    )


def collect_blackboard_key_usage_from_nodes(
    state_nodes: Iterable[object],
    metadata_map: Dict[str, Dict[str, str]],
    resolve_effective_name: Callable[[object, str], str],
) -> Dict[str, Dict[str, str]]:
    """Aggregate blackboard usage from rendered state nodes.

    This helper is used for the currently visible container where plugin metadata
    is attached to node items rather than resolved from the model tree.
    """

    usage_map: Dict[str, Dict[str, object]] = {}

    def add_usage(key_name: str, usage_kind: str, description: str) -> None:
        entry = usage_map.setdefault(
            key_name,
            {"input": False, "output": False, "description": ""},
        )
        entry[usage_kind] = True
        if not entry["description"] and description:
            entry["description"] = description

    for state_node in state_nodes:
        plugin_info = getattr(state_node, "plugin_info", None)
        if plugin_info is None:
            continue

        for key_info in list(getattr(plugin_info, "input_keys", []) or []):
            key_name = str(key_info.get("name", "")).strip()
            if not key_name:
                continue
            effective_name = resolve_effective_name(state_node, key_name)
            if not effective_name:
                continue
            add_usage(
                effective_name,
                "input",
                str(key_info.get("description", "") or "").strip(),
            )

        for key_info in list(getattr(plugin_info, "output_keys", []) or []):
            key_name = str(key_info.get("name", "")).strip()
            if not key_name:
                continue
            effective_name = resolve_effective_name(state_node, key_name)
            if not effective_name:
                continue
            add_usage(
                effective_name,
                "output",
                str(key_info.get("description", "") or "").strip(),
            )

    return _build_derived_keys(usage_map, metadata_map)


def _build_derived_keys(
    usage_map: Dict[str, Dict[str, object]],
    metadata_map: Dict[str, Dict[str, str]],
) -> Dict[str, Dict[str, str]]:
    """Merge observed usage flags with stored metadata."""

    derived_keys: Dict[str, Dict[str, str]] = {}
    for key_name, usage in usage_map.items():
        metadata = dict(metadata_map.get(key_name, {}))
        if usage["input"] and usage["output"]:
            key_type = "in/out"
        elif usage["output"]:
            key_type = "out"
        else:
            key_type = "in"

        description = str(metadata.get("description", "") or "").strip()
        if not description:
            description = str(usage.get("description", "") or "").strip()

        default_type = ""
        default_value = ""
        if key_type in ("in", "in/out"):
            default_type = str(metadata.get("default_type", "") or "")
            if default_type:
                default_value = str(metadata.get("default_value", "") or "")

        derived_keys[key_name] = {
            "name": key_name,
            "key_type": key_type,
            "description": description,
            "default_type": default_type,
            "default_value": default_value,
        }

    return dict(sorted(derived_keys.items(), key=lambda item: item[0].lower()))


def collect_blackboard_key_usage_for_model(
    container_model: StateMachine | Concurrence,
    resolve_plugin_info_for_model: ResolvePluginInfo,
) -> Tuple[Dict[str, Dict[str, str]], set[str]]:
    """Collect derived blackboard usage and hidden remapped names for one container."""

    usage_map: Dict[str, Dict[str, object]] = {}
    metadata_map = build_container_metadata_map(container_model)
    hidden_key_names: set[str] = set()

    def add_usage(key_name: str, usage_kind: str, description: str) -> None:
        entry = usage_map.setdefault(
            key_name,
            {"input": False, "output": False, "description": ""},
        )
        entry[usage_kind] = True
        if not entry["description"] and description:
            entry["description"] = description

    def resolve_local_name(raw_name: str, remap_chain: List[Dict[str, str]]) -> str:
        effective_name = raw_name
        intermediate_names: List[str] = []
        for remappings in remap_chain:
            intermediate_names.append(effective_name)
            effective_name = remappings.get(effective_name, effective_name)
        hidden_key_names.update(
            name for name in intermediate_names if name and name != effective_name
        )
        return effective_name

    def visit_state(state_model: State, ancestor_remaps: List[Dict[str, str]]) -> None:
        state_remaps = [dict(state_model.remappings)] + ancestor_remaps
        if isinstance(state_model, (StateMachine, Concurrence)):
            for child_state in state_model.states.values():
                visit_state(child_state, state_remaps)
            return

        try:
            plugin_info = resolve_plugin_info_for_model(state_model)
        except Exception:
            return

        for key_info in list(getattr(plugin_info, "input_keys", []) or []):
            raw_name = str(key_info.get("name", "")).strip()
            if not raw_name:
                continue
            add_usage(
                resolve_local_name(raw_name, state_remaps),
                "input",
                str(key_info.get("description", "") or "").strip(),
            )

        for key_info in list(getattr(plugin_info, "output_keys", []) or []):
            raw_name = str(key_info.get("name", "")).strip()
            if not raw_name:
                continue
            add_usage(
                resolve_local_name(raw_name, state_remaps),
                "output",
                str(key_info.get("description", "") or "").strip(),
            )

    for child_state in container_model.states.values():
        visit_state(child_state, [])

    return _build_derived_keys(usage_map, metadata_map), hidden_key_names


def has_persistent_blackboard_metadata(metadata: Dict[str, str]) -> bool:
    """Return whether one metadata entry should survive without live usage."""

    return any(
        str(metadata.get(field, "") or "").strip()
        for field in ["default_type", "default_value"]
    )


def merge_container_keys(
    container_model: StateMachine | Concurrence,
    resolve_plugin_info_for_model: ResolvePluginInfo,
) -> Dict[str, Dict[str, str]]:
    """Merge derived usage and persistent metadata for one container."""

    derived_keys, hidden_key_names = collect_blackboard_key_usage_for_model(
        container_model,
        resolve_plugin_info_for_model,
    )
    metadata_map = build_container_metadata_map(container_model)
    merged: Dict[str, Dict[str, str]] = {}

    for key_name in sorted(
        set(derived_keys.keys()) | set(metadata_map.keys()),
        key=str.lower,
    ):
        metadata = dict(metadata_map.get(key_name, {}))
        if key_name not in derived_keys:
            if key_name in hidden_key_names:
                continue
            if not has_persistent_blackboard_metadata(metadata):
                continue

        derived = dict(derived_keys.get(key_name, {}))
        key_type = str(derived.get("key_type", metadata.get("key_type", "in")) or "in")
        merged[key_name] = {
            "name": key_name,
            "key_type": key_type,
            "description": str(
                metadata.get("description", "") or derived.get("description", "") or ""
            ),
            "default_type": str(
                metadata.get("default_type", "") or derived.get("default_type", "") or ""
            ),
            "default_value": str(
                metadata.get("default_value", "")
                or derived.get("default_value", "")
                or ""
            ),
        }
    return merged


def state_uses_blackboard_key(
    state_model: Optional[State],
    key_name: str,
    resolve_plugin_info_for_model: ResolvePluginInfo,
) -> bool:
    """Return whether one state subtree uses the effective blackboard key name."""

    def model_uses_key(state_model: State, remap_chain: List[Dict[str, str]]) -> bool:
        current_chain = remap_chain + [dict(getattr(state_model, "remappings", {}) or {})]

        for key in getattr(state_model, "keys", []) or []:
            raw_name = str(getattr(key, "name", "") or "").strip()
            if (
                raw_name
                and get_effective_blackboard_key_name(current_chain, raw_name) == key_name
            ):
                return True

        if isinstance(state_model, (StateMachine, Concurrence)):
            for child_state in state_model.states.values():
                if model_uses_key(child_state, current_chain):
                    return True
            return False

        try:
            plugin_info = resolve_plugin_info_for_model(state_model)
        except Exception:
            return False

        for key_info in list(getattr(plugin_info, "input_keys", []) or []) + list(
            getattr(plugin_info, "output_keys", []) or []
        ):
            plugin_key_name = str(key_info.get("name", "")).strip()
            if (
                plugin_key_name
                and get_effective_blackboard_key_name(current_chain, plugin_key_name)
                == key_name
            ):
                return True
        return False

    if state_model is None:
        return False
    return model_uses_key(state_model, [])


def collect_container_key_lists(
    model: State,
) -> Tuple[List[Dict[str, str]], List[Dict[str, str]]]:
    """Return separate input and output key tables for a container edit dialog."""

    input_keys: List[Dict[str, str]] = []
    output_keys: List[Dict[str, str]] = []
    for key in getattr(model, "keys", []) or []:
        key_data = {
            "name": str(getattr(key, "name", "") or ""),
            "description": str(getattr(key, "description", "") or ""),
            "default_type": str(getattr(key, "default_type", "") or ""),
            "default_value": (
                ""
                if getattr(key, "default_value", None) is None
                else str(getattr(key, "default_value", ""))
            ),
            "has_default": bool(getattr(key, "default_type", "") or ""),
        }
        key_type = str(getattr(key, "key_type", "in") or "in")
        if key_type in ("in", "in/out"):
            input_keys.append(dict(key_data))
        if key_type in ("out", "in/out"):
            output_keys.append(dict(key_data))
    return input_keys, output_keys
