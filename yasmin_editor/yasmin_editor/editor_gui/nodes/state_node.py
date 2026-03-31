# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import Any, Dict, List, Optional

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QBrush, QFont, QPen
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsItem, QGraphicsTextItem
from yasmin_plugins_manager.plugin_info import PluginInfo

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.state import State


class StateNode(QGraphicsEllipseItem, BaseNodeMixin):
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

        self._initialize_base_node_graphics(x, y)

        self.setBrush(
            QBrush(PALETTE.state_fill(plugin_info.plugin_type if plugin_info else None))
        )
        self.setPen(QPen(PALETTE.state_pen, 3))

        self.text: QGraphicsTextItem = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(PALETTE.text_primary)
        font: QFont = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

        if plugin_info:
            type_text: str = plugin_info.plugin_type.upper()
            self.type_label: QGraphicsTextItem = QGraphicsTextItem(type_text, self)
            self.type_label.setDefaultTextColor(PALETTE.text_secondary)
            type_font: QFont = QFont()
            type_font.setPointSize(8)
            self.type_label.setFont(type_font)
            self.center_text_item(self.type_label, 10)

        self.connection_port: ConnectionPort = ConnectionPort(self)

    @property
    def name(self) -> str:
        return self.model.name

    @name.setter
    def name(self, value: str) -> None:
        self.model.name = value
        if hasattr(self, "text"):
            self.text.setPlainText(value)
            self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

    @property
    def description(self) -> str:
        return self.model.description

    @description.setter
    def description(self, value: str) -> None:
        self.model.description = value

    @property
    def remappings(self) -> Dict[str, str]:
        return self.model.remappings

    @remappings.setter
    def remappings(self, value: Dict[str, str]) -> None:
        self.model.remappings.clear()
        self.model.remappings.update(value or {})

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Handle double-click to edit state."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                canvas.editor_ref.edit_state()
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            value = self.constrain_position_to_parent(value)
            self.update_attached_connections()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            self.notify_parent_container_resized()

        elif change == QGraphicsItem.ItemSelectedChange:
            self.update_selection_pen(bool(value), QPen(PALETTE.state_pen, 3))

        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
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
        return QPointF(center.x() + dx * scale, center.y() + dy * scale)
