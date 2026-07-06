# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import Any, Dict, List, Optional, Union

from yasmin_plugins_manager.plugin_info import PluginInfo
from yasmin_editor.qt_compat import QtCore, QtGui, QtWidgets, exec_menu
from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.state import State


class StateNode(BaseNodeMixin, QtWidgets.QGraphicsEllipseItem):
    """Graphical representation of a regular state (not container)."""

    def __init__(
        self,
        name: str,
        plugin_info: PluginInfo,
        x: float,
        y: float,
        remappings: Optional[Dict[str, str]] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
        custom_type_label: Union[str, None] = None,
        model: Optional[State] = None,
    ) -> None:
        super().__init__(-60, -40, 120, 80)
        self.model: State = model or State(
            name=name,
            description=description,
            remappings=dict(remappings or {}),
        )
        self.plugin_info: PluginInfo = plugin_info
        self.is_state_machine: bool = False
        self.is_concurrence: bool = False
        self.defaults: List[Dict[str, str]] = defaults or []
        self.custom_type_label: Union[str, None] = custom_type_label

        self._initialize_base_node_graphics(x, y)

        self.setBrush(
            QtGui.QBrush(
                PALETTE.state_fill(plugin_info.plugin_type if plugin_info else None)
            )
        )
        self.setPen(QtGui.QPen(PALETTE.state_pen, 3))

        self.text: QtWidgets.QGraphicsTextItem = QtWidgets.QGraphicsTextItem(
            self.name, self
        )
        self.text.setDefaultTextColor(PALETTE.text_primary)
        font: QtGui.QFont = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

        type_text: Union[str, None] = None
        if plugin_info:
            type_text = plugin_info.plugin_type.upper()
        elif custom_type_label:
            type_text = custom_type_label

        if type_text:
            self.type_label: QtWidgets.QGraphicsTextItem = QtWidgets.QGraphicsTextItem(
                type_text, self
            )
            self.type_label.setDefaultTextColor(PALETTE.text_secondary)
            type_font: QtGui.QFont = QtGui.QFont()
            type_font.setPointSize(8)
            self.type_label.setFont(type_font)
            self.center_text_item(self.type_label, 10)

        self.connection_port: ConnectionPort = ConnectionPort(self)
        self.initialize_breakpoint_marker()
        self.initialize_start_indicator()

    def _on_name_changed(self, value: str) -> None:
        if hasattr(self, "text"):
            self.text.setPlainText(value)
            self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

    def _on_double_click(self, editor: Any, event: Any) -> None:
        editor.edit_state()

    def _on_context_menu(self, editor: Any, event: Any) -> bool:
        if getattr(editor, "runtime_mode_enabled", False):
            if editor.show_runtime_breakpoint_menu(self, event.screenPos()):
                return True

        if editor.is_read_only_mode():
            menu = QtWidgets.QMenu()
            view_action = menu.addAction("View Properties")
            action = exec_menu(menu, event.screenPos())
            if action == view_action:
                editor.edit_state()
                return True
        else:
            menu = QtWidgets.QMenu()
            edit_action = menu.addAction("Edit Properties")
            delete_action = menu.addAction("Delete")
            action = exec_menu(menu, event.screenPos())
            if action == edit_action:
                editor.edit_state()
                return True
            if action == delete_action:
                editor.delete_selected()
                return True
        return False

    def itemChange(
        self, change: QtWidgets.QGraphicsItem.GraphicsItemChange, value: Any
    ) -> Any:
        if (
            change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and isinstance(value, QtCore.QPointF)
        ):
            value = self.constrain_position_to_parent(value)
            self.update_attached_connections()

        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.notify_parent_container_resized()

        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedChange:
            self.update_selection_pen(bool(value), QtGui.QPen(PALETTE.state_pen, 3))

        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QtCore.QPointF) -> QtCore.QPointF:
        """Get the ellipse boundary point on the ray from the center to the target."""
        bounds = self.sceneBoundingRect()
        center = bounds.center()
        rx = max(bounds.width() / 2.0, 1.0)
        ry = max(bounds.height() / 2.0, 1.0)
        dx = target_pos.x() - center.x()
        dy = target_pos.y() - center.y()

        denominator = ((dx * dx) / (rx * rx)) + ((dy * dy) / (ry * ry))
        if denominator <= 1e-9:
            return center

        scale = 1.0 / (denominator**0.5)
        return QtCore.QPointF(center.x() + dx * scale, center.y() + dy * scale)
