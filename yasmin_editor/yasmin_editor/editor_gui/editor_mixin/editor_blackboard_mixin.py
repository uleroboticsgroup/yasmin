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

from yasmin_editor.editor_gui.blackboard_logic import (
    build_container_metadata_map,
    collect_blackboard_key_usage_for_model,
    collect_blackboard_key_usage_from_nodes,
    collect_container_key_lists,
    dicts_to_keys as build_key_models,
    format_blackboard_key_label,
    get_effective_blackboard_key_name,
    has_persistent_blackboard_metadata,
    keys_to_dicts as build_key_rows,
    merge_container_keys,
    metadata_map_to_keys,
    should_hide_blackboard_key,
    state_uses_blackboard_key as model_state_uses_blackboard_key,
)
from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import BlackboardKeyDialog
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine


class EditorBlackboardMixin:
    """Mixin for editor functionality split from the main window."""

    def filter_blackboard_keys(self, text: str) -> None:
        """Filter blackboard keys based on search text and visibility settings."""
        show_hidden = getattr(self, "_show_hidden_blackboard_keys", False)

        for i in range(self.blackboard_list.count()):
            item = self.blackboard_list.item(i)
            key_data = item.data(Qt.UserRole) or {}
            item.setHidden(
                should_hide_blackboard_key(
                    item_text=item.text(),
                    key_name=str(key_data.get("name", "") or ""),
                    filter_text=text,
                    show_hidden=show_hidden,
                )
            )

    def format_blackboard_key_label(self, key_data: Dict[str, str]) -> str:
        """Return the visible list label for one blackboard key."""

        return format_blackboard_key_label(key_data)

    def dicts_to_keys(self, keys: List[Dict[str, str]]) -> List[Key]:
        """Normalize editor rows into key models."""

        return build_key_models(keys)

    def keys_to_dicts(self, keys: List[Key]) -> List[Dict[str, str]]:
        """Convert key models into editor rows."""

        return build_key_rows(keys)

    def _collect_blackboard_key_usage(self) -> Dict[str, Dict[str, str]]:
        """Collect usage for the currently rendered container from scene nodes."""

        return collect_blackboard_key_usage_from_nodes(
            self.state_nodes.values(),
            dict(self._blackboard_key_metadata),
            self.get_effective_blackboard_key_name,
        )

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

    def toggle_hidden_blackboard_keys(self, enabled: bool) -> None:
        self._show_hidden_blackboard_keys = enabled
        self.show_hidden_blackboard_btn.setText(
            "Hidden: On" if enabled else "Hidden: Off"
        )
        self.filter_blackboard_keys(self.blackboard_filter.text())

    def get_effective_blackboard_key_name(self, state_node, key_name: str) -> str:
        """Return one visible key name after applying nested node remappings."""

        remap_chain = []
        current_node = state_node
        while current_node is not None:
            remap_chain.append(getattr(current_node, "remappings", {}) or {})
            current_node = getattr(current_node, "parent_container", None)

        return get_effective_blackboard_key_name(remap_chain, key_name)

    def _get_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
        """Return persisted key metadata for one container model."""

        return build_container_metadata_map(container_model)

    def _set_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
        metadata: Dict[str, Dict[str, str]],
    ) -> None:
        """Persist one normalized metadata map back onto the container."""

        container_model.keys = metadata_map_to_keys(metadata)

    def _collect_blackboard_key_usage_for_model(
        self,
        container_model: StateMachine | Concurrence,
    ) -> tuple[Dict[str, Dict[str, str]], set[str]]:
        """Collect derived key usage for one container model tree."""

        return collect_blackboard_key_usage_for_model(
            container_model,
            self.resolve_plugin_info_for_model,
        )

    def _has_persistent_blackboard_metadata(
        self,
        metadata: Dict[str, str],
    ) -> bool:
        """Return whether metadata should remain without live usage."""

        return has_persistent_blackboard_metadata(metadata)

    def _merge_container_keys(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
        """Return merged live-usage and persistent key metadata."""

        return merge_container_keys(container_model, self.resolve_plugin_info_for_model)

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

    def set_blackboard_keys(self, keys: List[Dict[str, str]], sync: bool = True) -> None:

        self.root_model.keys = self.dicts_to_keys(keys)
        self._blackboard_key_metadata = self._get_container_metadata_map(
            self.current_container_model
        )
        self._blackboard_keys = [dict(item) for item in keys]
        if sync:
            self.sync_blackboard_keys()
            self.record_history_checkpoint()
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
        self.record_history_checkpoint()

    def state_uses_blackboard_key(self, state_node, key_name: str) -> bool:
        """Return whether one rendered state subtree uses the selected key."""

        model = getattr(state_node, "model", None)
        return model_state_uses_blackboard_key(
            model,
            key_name,
            self.resolve_plugin_info_for_model,
        )

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
            metadata_map = self._get_container_metadata_map(self.current_container_model)
            metadata_map[key_name] = {
                "description": updated_key.get("description", ""),
                "key_type": key_data.get("key_type", "in"),
                "default_type": updated_key.get("default_type", ""),
                "default_value": updated_key.get("default_value", ""),
            }
            self._set_container_metadata_map(self.current_container_model, metadata_map)
            self.sync_blackboard_keys()
            self.record_history_checkpoint()

    def _collect_container_key_lists(
        self, model: State
    ) -> tuple[List[Dict[str, str]], List[Dict[str, str]]]:
        """Return separate input and output key rows for one container."""

        return collect_container_key_lists(model)
