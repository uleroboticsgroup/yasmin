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

import math
from typing import Dict, List
from PyQt5.QtWidgets import (
    QGraphicsItem,
    QGraphicsEllipseItem,
    QGraphicsTextItem,
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont

from yasmin_editor.plugins_manager.plugin_info import PluginInfo
from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.connection_line import ConnectionLine


class StateNode(QGraphicsEllipseItem):
    """Graphical representation of a regular state (not container)."""

    def __init__(
        self,
        name: str,
        plugin_info: PluginInfo,
        x: float,
        y: float,
        remappings: Dict[str, str] = None,
    ):
        super().__init__(-60, -40, 120, 80)
        self.name = name
        self.plugin_info = plugin_info
        self.is_state_machine = False
        self.is_concurrence = False
        self.connections: List["ConnectionLine"] = []
        self.remappings = remappings or {}
        self.parent_container = None  # Reference to parent container if nested

        # Set position
        self.setPos(x, y)

        # Set flags
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        # Set colors based on type
        if plugin_info and plugin_info.plugin_type == "python":
            self.setBrush(QBrush(QColor(144, 238, 144)))  # Light green
        elif plugin_info and plugin_info.plugin_type == "cpp":
            self.setBrush(QBrush(QColor(255, 182, 193)))  # Light pink
        else:
            self.setBrush(QBrush(QColor(255, 165, 0)))  # Light orange

        self.setPen(QPen(QColor(0, 0, 0), 2))

        # Add text label
        self.text = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(Qt.black)
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        # Center text
        text_rect = self.text.boundingRect()
        self.text.setPos(-text_rect.width() / 2, -text_rect.height() / 2)

        # Add type label
        if plugin_info:
            type_text = plugin_info.plugin_type.upper()
        else:
            type_text = ""

        if type_text:
            self.type_label = QGraphicsTextItem(type_text, self)
            self.type_label.setDefaultTextColor(Qt.darkGray)
            type_font = QFont()
            type_font.setPointSize(8)
            self.type_label.setFont(type_font)
            type_rect = self.type_label.boundingRect()
            self.type_label.setPos(-type_rect.width() / 2, 10)

        # Add connection port (small circle on the right edge for drag-to-connect)
        self.connection_port = ConnectionPort(self)

    def mouseDoubleClickEvent(self, event):
        """Handle double-click to edit state."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                # Select this item first
                self.setSelected(True)
                canvas.editor_ref.edit_state()
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            # If this state is inside a container, constrain its movement
            if self.parent_container:
                # Get container's bounds in local coordinates
                container_rect = self.parent_container.rect()
                state_rect = self.boundingRect()

                # Calculate the constrained position
                new_pos = value

                # Ensure state stays within container bounds
                min_x = container_rect.left() - state_rect.left() + 10
                max_x = container_rect.right() - state_rect.right() - 10
                min_y = (
                    container_rect.top() - state_rect.top() + 40
                )  # Extra space for header
                max_y = container_rect.bottom() - state_rect.bottom() - 10

                # Clamp position
                constrained_x = max(min_x, min(new_pos.x(), max_x))
                constrained_y = max(min_y, min(new_pos.y(), max_y))

                value = QPointF(constrained_x, constrained_y)

            # Update all connections when the state moves
            for connection in self.connections:
                connection.update_position()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            # After position has changed, trigger parent resize if in a container
            if self.parent_container:
                self.parent_container.auto_resize_for_children()
        
        elif change == QGraphicsItem.ItemSelectedChange:
            # Highlight selected items in yellow
            if value:  # Selected
                self.setPen(QPen(QColor(255, 200, 0), 4))  # Yellow/orange highlight
            else:  # Deselected
                self.setPen(QPen(QColor(0, 0, 180), 3))  # Original blue

        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine"):
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine"):
        if connection in self.connections:
            self.connections.remove(connection)

    def get_used_outcomes(self):
        """Get set of outcomes that already have connections."""
        used_outcomes = set()
        for connection in self.connections:
            used_outcomes.add(connection.from_node.name + connection.outcome)
        return used_outcomes

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach (center of port)."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the ellipse edge closest to target."""
        center = self.scenePos()
        angle = math.atan2(target_pos.y() - center.y(), target_pos.x() - center.x())
        # Ellipse dimensions
        rx = 60
        ry = 40
        # Point on ellipse
        x = center.x() + rx * math.cos(angle)
        y = center.y() + ry * math.sin(angle)
        return QPointF(x, y)
