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

from typing import Dict, List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidgetItem

from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import \
    BlackboardKeyDialog
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine


class EditorBlackboardMixin:
    """Mixin for editor functionality split from the main window."""

    def filter_blackboard_keys(self, text: str) -> None:
        """Filter blackboard keys based on search text."""
        for i in range(self.blackboard_list.count()):
            item = self.blackboard_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def format_blackboard_key_label(self, key_data: Dict[str, str]) -> str:
        label = f"{key_data.get('name', '')} ({key_data.get('key_type', 'in')})"

        default_type = str(key_data.get("default_type", "")).strip()
        if default_type:
            default_value = str(key_data.get("default_value", ""))
            label += f" [default: {default_value}, type: {default_type}]"

        return label

    def dicts_to_keys(self, keys: List[Dict[str, str]]) -> List[Key]:
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

    def keys_to_dicts(self, keys: List[Key]) -> List[Dict[str, str]]:
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

    def _get_plugin_key_usage(
        self, state_node: StateNode, key_info: Dict[str, str], is_input: bool
    ) -> Optional[Dict[str, str]]:
        key_name = str(key_info.get("name", "")).strip()
        if not key_name:
            return None

        effective_name = self.get_effective_blackboard_key_name(state_node, key_name)
        if not effective_name:
            return None

        description = str(key_info.get("description", "") or "").strip()
        return {
            "name": effective_name,
            "usage": "input" if is_input else "output",
            "description": description,
        }

    def _collect_blackboard_key_usage(self) -> Dict[str, Dict[str, str]]:
        usage_map: Dict[str, Dict[str, object]] = {}

        for state_node in self.state_nodes.values():
            plugin_info = getattr(state_node, "plugin_info", None)
            if plugin_info is None:
                continue

            for key_info in list(getattr(plugin_info, "input_keys", []) or []):
                usage = self._get_plugin_key_usage(state_node, key_info, True)
                if usage is None:
                    continue
                entry = usage_map.setdefault(
                    usage["name"],
                    {
                        "input": False,
                        "output": False,
                        "description": "",
                    },
                )
                entry["input"] = True
                if not entry["description"] and usage["description"]:
                    entry["description"] = usage["description"]

            for key_info in list(getattr(plugin_info, "output_keys", []) or []):
                usage = self._get_plugin_key_usage(state_node, key_info, False)
                if usage is None:
                    continue
                entry = usage_map.setdefault(
                    usage["name"],
                    {
                        "input": False,
                        "output": False,
                        "description": "",
                    },
                )
                entry["output"] = True
                if not entry["description"] and usage["description"]:
                    entry["description"] = usage["description"]

        derived_keys: Dict[str, Dict[str, str]] = {}
        for key_name, usage in usage_map.items():
            metadata = dict(self._blackboard_key_metadata.get(key_name, {}))
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

    def get_selected_blackboard_key_name(self) -> Optional[str]:
        item = self.blackboard_list.currentItem()
        if item is None:
            return None
        key_data = item.data(Qt.UserRole) or {}
        return key_data.get("name")

    def on_blackboard_selection_changed(self) -> None:
        self.update_blackboard_usage_highlighting()

    def toggle_blackboard_highlighting(self, enabled: bool) -> None:
        self._highlight_blackboard_usage = enabled
        self.highlight_blackboard_btn.setText(
            "Highlight: On" if enabled else "Highlight: Off"
        )
        self.update_blackboard_usage_highlighting()

    def get_effective_blackboard_key_name(self, state_node, key_name: str) -> str:
        effective_key_name = key_name

        remap_chain = []
        current_node = state_node
        while current_node is not None:
            remap_chain.append(getattr(current_node, "remappings", {}) or {})
            current_node = getattr(current_node, "parent_container", None)

        for remappings in remap_chain:
            effective_key_name = remappings.get(effective_key_name, effective_key_name)

        return effective_key_name

    def _get_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
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

    def _set_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
        metadata: Dict[str, Dict[str, str]],
    ) -> None:
        container_model.keys = self.dicts_to_keys(
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

    def _collect_blackboard_key_usage_for_model(
        self,
        container_model: StateMachine | Concurrence,
    ) -> tuple[Dict[str, Dict[str, str]], set[str]]:
        usage_map: Dict[str, Dict[str, object]] = {}
        metadata_map = self._get_container_metadata_map(container_model)
        hidden_key_names: set[str] = set()

        def add_usage(key_name: str, usage_kind: str, description: str) -> None:
            entry = usage_map.setdefault(
                key_name,
                {
                    "input": False,
                    "output": False,
                    "description": "",
                },
            )
            entry[usage_kind] = True
            if not entry["description"] and description:
                entry["description"] = description

        def resolve_local_name(
            raw_name: str,
            remap_chain: List[Dict[str, str]],
        ) -> str:
            effective_name = raw_name
            intermediate_names: list[str] = []

            for remappings in remap_chain:
                intermediate_names.append(effective_name)
                effective_name = remappings.get(effective_name, effective_name)

            hidden_key_names.update(
                name for name in intermediate_names if name and name != effective_name
            )
            return effective_name

        def visit_state(
            state_model: State, ancestor_remaps: List[Dict[str, str]]
        ) -> None:
            state_remaps = [dict(state_model.remappings)] + ancestor_remaps

            if isinstance(state_model, (StateMachine, Concurrence)):
                for child_state in state_model.states.values():
                    visit_state(child_state, state_remaps)
                return

            try:
                plugin_info = self.resolve_plugin_info_for_model(state_model)
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

        return (
            dict(sorted(derived_keys.items(), key=lambda item: item[0].lower())),
            hidden_key_names,
        )

    def _has_persistent_blackboard_metadata(
        self,
        metadata: Dict[str, str],
    ) -> bool:
        return any(
            str(metadata.get(field, "") or "").strip()
            for field in ["default_type", "default_value"]
        )

    def _merge_container_keys(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
        derived_keys, hidden_key_names = self._collect_blackboard_key_usage_for_model(
            container_model
        )
        metadata_map = self._get_container_metadata_map(container_model)
        merged: Dict[str, Dict[str, str]] = {}

        for key_name in sorted(
            set(derived_keys.keys()) | set(metadata_map.keys()), key=str.lower
        ):
            metadata = dict(metadata_map.get(key_name, {}))
            if key_name not in derived_keys:
                if key_name in hidden_key_names:
                    continue
                if not self._has_persistent_blackboard_metadata(metadata):
                    continue

            derived = dict(derived_keys.get(key_name, {}))
            key_type = str(
                derived.get("key_type", metadata.get("key_type", "in")) or "in"
            )
            merged[key_name] = {
                "name": key_name,
                "key_type": key_type,
                "description": str(
                    metadata.get("description", "")
                    or derived.get("description", "")
                    or ""
                ),
                "default_type": str(
                    metadata.get("default_type", "")
                    or derived.get("default_type", "")
                    or ""
                ),
                "default_value": str(
                    metadata.get("default_value", "")
                    or derived.get("default_value", "")
                    or ""
                ),
            }
        return merged

    def _rebuild_root_blackboard_keys(self) -> None:
        merged_root = self._merge_container_keys(self.root_model)
        self.root_model.keys = self.dicts_to_keys(list(merged_root.values()))

    def sync_blackboard_keys(self) -> None:
        current_container = self.current_container_model
        merged = self._merge_container_keys(current_container)
        self._set_container_metadata_map(current_container, merged)
        self._blackboard_keys = list(merged.values())
        self._blackboard_key_metadata = {
            item["name"]: {
                "description": item.get("description", ""),
                "key_type": item.get("key_type", "in"),
                "default_type": item.get("default_type", ""),
                "default_value": item.get("default_value", ""),
            }
            for item in self._blackboard_keys
        }
        self._rebuild_root_blackboard_keys()
        self.refresh_blackboard_keys_list()

    def refresh_blackboard_keys_list(self) -> None:
        current_key_name = self.get_selected_blackboard_key_name()
        merged = self._merge_container_keys(self.current_container_model)
        self._blackboard_key_metadata = {
            key_name: {
                "description": values.get("description", ""),
                "key_type": values.get("key_type", "in"),
                "default_type": values.get("default_type", ""),
                "default_value": values.get("default_value", ""),
            }
            for key_name, values in merged.items()
        }
        self._blackboard_keys = list(merged.values())
        self.blackboard_list.clear()

        for key_data in sorted(
            self._blackboard_keys, key=lambda item: item.get("name", "").lower()
        ):
            item = QListWidgetItem(self.format_blackboard_key_label(key_data))
            item.setData(Qt.UserRole, dict(key_data))
            description = key_data.get("description", "")
            if description:
                item.setToolTip(description)
            self.blackboard_list.addItem(item)

        self.filter_blackboard_keys(self.blackboard_filter.text())

        if current_key_name:
            for i in range(self.blackboard_list.count()):
                item = self.blackboard_list.item(i)
                key_data = item.data(Qt.UserRole) or {}
                if key_data.get("name") == current_key_name:
                    self.blackboard_list.setCurrentItem(item)
                    break

        self.update_blackboard_usage_highlighting()

    def set_blackboard_keys(
        self, keys: List[Dict[str, str]], sync: bool = True
    ) -> None:

        self.root_model.keys = self.dicts_to_keys(keys)
        self._blackboard_key_metadata = self._get_container_metadata_map(
            self.current_container_model
        )
        self._blackboard_keys = [dict(item) for item in keys]
        if sync:
            self.sync_blackboard_keys()
        else:
            self.refresh_blackboard_keys_list()

    def get_blackboard_keys(self) -> List[Dict[str, str]]:
        self.sync_blackboard_keys()
        return self.keys_to_dicts(self.current_container_model.keys)

    def add_root_default_row_with_data(self, data: dict) -> None:
        key_name = str(data.get("key", "") or "").strip()
        if not key_name:
            return
        metadata = self._get_container_metadata_map(self.current_container_model)
        metadata[key_name] = {
            "description": str(data.get("description", "") or "").strip(),
            "key_type": "in",
            "default_type": str(data.get("type", "") or "").strip(),
            "default_value": str(data.get("value", "") or "").strip(),
        }
        self._set_container_metadata_map(self.current_container_model, metadata)
        self.sync_blackboard_keys()

    def state_uses_blackboard_key(self, state_node, key_name: str) -> bool:
        def apply_remap_chain(raw_name: str, remap_chain: List[Dict[str, str]]) -> str:
            effective_name = raw_name
            for remappings in remap_chain:
                effective_name = remappings.get(effective_name, effective_name)
            return effective_name

        def model_uses_key(
            state_model: State, remap_chain: List[Dict[str, str]]
        ) -> bool:
            current_chain = remap_chain + [
                dict(getattr(state_model, "remappings", {}) or {})
            ]

            for key in getattr(state_model, "keys", []) or []:
                raw_name = str(getattr(key, "name", "") or "").strip()
                if raw_name and apply_remap_chain(raw_name, current_chain) == key_name:
                    return True

            if isinstance(state_model, (StateMachine, Concurrence)):
                for child_state in state_model.states.values():
                    if model_uses_key(child_state, current_chain):
                        return True
                return False

            try:
                plugin_info = self.resolve_plugin_info_for_model(state_model)
            except Exception:
                return False

            for key_info in list(getattr(plugin_info, "input_keys", []) or []) + list(
                getattr(plugin_info, "output_keys", []) or []
            ):
                plugin_key_name = str(key_info.get("name", "")).strip()
                if (
                    plugin_key_name
                    and apply_remap_chain(plugin_key_name, current_chain) == key_name
                ):
                    return True
            return False

        model = getattr(state_node, "model", None)
        if model is None:
            return False
        return model_uses_key(model, [])

    def update_blackboard_usage_highlighting(self) -> None:
        self.refresh_visual_highlighting()

    def edit_selected_blackboard_key(
        self, item: Optional[QListWidgetItem] = None
    ) -> None:
        if item is None:
            item = self.blackboard_list.currentItem()
        if item is None:
            return

        key_data = dict(item.data(Qt.UserRole) or {})
        key_name = key_data.get("name", "")
        if not key_name:
            return

        metadata = dict(
            self._get_container_metadata_map(self.current_container_model).get(
                key_name, {}
            )
        )
        merged_key_data = {
            **key_data,
            **metadata,
            "name": key_data.get("name", ""),
            "key_type": key_data.get("key_type", "in"),
            "default_type": str(metadata.get("default_type", "") or ""),
            "default_value": str(metadata.get("default_value", "") or ""),
        }

        dlg = BlackboardKeyDialog(
            merged_key_data,
            parent=self,
            edit_mode=True,
            readonly=self.is_read_only_mode(),
        )
        if self.is_read_only_mode():
            dlg.exec_()
            return
        if dlg.exec_():
            updated_key = dlg.get_key_data()
            metadata_map = self._get_container_metadata_map(
                self.current_container_model
            )
            metadata_map[key_name] = {
                "description": updated_key.get("description", ""),
                "key_type": key_data.get("key_type", "in"),
                "default_type": updated_key.get("default_type", ""),
                "default_value": updated_key.get("default_value", ""),
            }
            self._set_container_metadata_map(self.current_container_model, metadata_map)
            self.sync_blackboard_keys()

    def _collect_container_key_lists(
        self, model: State
    ) -> tuple[List[Dict[str, str]], List[Dict[str, str]]]:
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
