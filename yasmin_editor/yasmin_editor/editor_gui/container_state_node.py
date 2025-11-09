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
    QGraphicsTextItem,
    QGraphicsRectItem,
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont

from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.connection_line import ConnectionLine


class ContainerStateNode(QGraphicsRectItem):
    """Container for State Machines and Concurrence states that can hold child states."""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        is_concurrence: bool = False,
        remappings: Dict[str, str] = None,
        outcomes: List[str] = None,
        initial_state: str = None,
        default_outcome: str = None,
    ):
        # Start with a default size, will expand as states are added
        super().__init__(-100, -80, 200, 160)
        self.name = name
        self.plugin_info = None  # Containers don't have plugin info
        self.is_state_machine = not is_concurrence
        self.is_concurrence = is_concurrence
        self.connections: List["ConnectionLine"] = []
        self.remappings = remappings or {}
        self.final_outcomes = (
            {}
        )  # Dict[str, FinalOutcomeNode] - final outcomes inside container
        self.initial_state = initial_state
        self.default_outcome = default_outcome  # For Concurrence
        self.child_states = {}  # Dict[str, Union[StateNode, ContainerStateNode]]
        self.parent_container = None
        self.min_width = 200
        self.min_height = 160
        self.xml_file = None  # For XML-based state machines

        # Set position
        self.setPos(x, y)

        # Set flags
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        # Set colors and style
        if is_concurrence:
            self.setBrush(QBrush(QColor(255, 220, 150, 180)))  # Semi-transparent orange
            self.setPen(QPen(QColor(255, 140, 0), 3))  # Dark orange border
        else:
            self.setBrush(
                QBrush(QColor(173, 216, 230, 180))
            )  # Semi-transparent light blue
            self.setPen(QPen(QColor(0, 0, 180), 3))  # Dark blue border

        # Add header background
        self.header = QGraphicsRectItem(self)
        self.header.setRect(-100, -80, 200, 30)
        if is_concurrence:
            self.header.setBrush(QBrush(QColor(255, 140, 0)))
        else:
            self.header.setBrush(QBrush(QColor(0, 100, 200)))
        self.header.setPen(QPen(Qt.NoPen))

        # Add title label
        self.title = QGraphicsTextItem(self)
        self.title.setDefaultTextColor(Qt.white)
        title_font = QFont()
        title_font.setPointSize(11)
        title_font.setBold(True)
        self.title.setFont(title_font)

        type_label = "CONCURRENCE" if is_concurrence else "STATE MACHINE"
        self.title.setPlainText(f"{type_label}: {name}")
        title_rect = self.title.boundingRect()
        self.title.setPos(-title_rect.width() / 2, -75)

        # Update initial state label (for SM only)
        if not is_concurrence:
            self.initial_state_label = QGraphicsTextItem(self)
            self.initial_state_label.setDefaultTextColor(QColor(0, 100, 0))
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.initial_state_label.setFont(label_font)
            self.update_initial_state_label()
        else:
            # Add default outcome label (for Concurrence only)
            self.default_outcome_label = QGraphicsTextItem(self)
            self.default_outcome_label.setDefaultTextColor(QColor(139, 69, 19))  # Brown
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.default_outcome_label.setFont(label_font)
            self.update_default_outcome_label()

        # Add connection port for drag-to-connect (containers can start connections)
        self.connection_port = ConnectionPort(self)

    def update_initial_state_label(self):
        """Update the initial state label text."""
        if not hasattr(self, "initial_state_label"):
            return
        if self.initial_state:
            self.initial_state_label.setPlainText(f"Initial: {self.initial_state}")
        else:
            self.initial_state_label.setPlainText("Initial: (none)")
        label_rect = self.initial_state_label.boundingRect()
        self.initial_state_label.setPos(-label_rect.width() / 2, -45)

    def update_default_outcome_label(self):
        """Update the default outcome label text for Concurrence."""
        if not hasattr(self, "default_outcome_label"):
            return
        if self.default_outcome:
            self.default_outcome_label.setPlainText(f"Default: {self.default_outcome}")
        else:
            self.default_outcome_label.setPlainText("Default: (none)")
        label_rect = self.default_outcome_label.boundingRect()
        self.default_outcome_label.setPos(-label_rect.width() / 2, -45)

    def add_child_state(self, state_node):
        """Add a child state to this container."""
        if state_node.name not in self.child_states:
            self.child_states[state_node.name] = state_node
            state_node.parent_container = self
            state_node.setParentItem(self)
            # Position relative to container
            if len(self.child_states) == 1:
                state_node.setPos(-50, -20)
            else:
                # Arrange in a grid
                index = len(self.child_states) - 1
                row = index // 2
                col = index % 2
                state_node.setPos(-80 + col * 100, -20 + row * 100)
            self.update_size()

    def remove_child_state(self, state_name: str):
        """Remove a child state from this container."""
        if state_name in self.child_states:
            state = self.child_states[state_name]
            state.parent_container = None
            state.setParentItem(None)
            del self.child_states[state_name]
            self.update_size()

    def update_size(self):
        """Adjust container size based on child states."""
        if not self.child_states:
            self.setRect(-100, -80, self.min_width, self.min_height)
            self.header.setRect(-100, -80, self.min_width, 30)
            return

        # Calculate bounding box of all children
        min_x, min_y = float("inf"), float("inf")
        max_x, max_y = float("-inf"), float("-inf")

        for child in self.child_states.values():
            child_rect = child.boundingRect()
            child_pos = child.pos()

            # Get child bounds
            left = child_pos.x() + child_rect.left()
            top = child_pos.y() + child_rect.top()
            right = child_pos.x() + child_rect.right()
            bottom = child_pos.y() + child_rect.bottom()

            min_x = min(min_x, left)
            min_y = min(min_y, top)
            max_x = max(max_x, right)
            max_y = max(max_y, bottom)

        # Add padding
        padding = 30
        min_x -= padding
        min_y -= padding + 50  # Extra for header
        max_x += padding
        max_y += padding

        # Ensure minimum size
        width = max(self.min_width, max_x - min_x)
        height = max(self.min_height, max_y - min_y)

        # Update rectangle
        self.setRect(min_x, min_y, width, height)
        self.header.setRect(min_x, min_y, width, 30)

        # Update title position
        title_rect = self.title.boundingRect()
        self.title.setPos(min_x + width / 2 - title_rect.width() / 2, min_y + 5)

        # Update initial state label position
        if hasattr(self, "initial_state_label"):
            label_rect = self.initial_state_label.boundingRect()
            self.initial_state_label.setPos(
                min_x + width / 2 - label_rect.width() / 2, min_y + 30
            )

        # Update connection port position if it exists
        if hasattr(self, "connection_port"):
            self.connection_port.setPos(min_x + width, min_y + height / 2)

    def mouseDoubleClickEvent(self, event):
        """Handle double-click to edit container state."""
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
            # If this container is inside another container, constrain its movement
            if self.parent_container:
                # Get parent container's bounds in local coordinates
                container_rect = self.parent_container.rect()
                self_rect = self.rect()

                # Calculate the constrained position
                new_pos = value

                # Ensure container stays within parent container bounds
                min_x = container_rect.left() - self_rect.left() + 10
                max_x = container_rect.right() - self_rect.right() - 10
                min_y = (
                    container_rect.top() - self_rect.top() + 40
                )  # Extra space for header
                max_y = container_rect.bottom() - self_rect.bottom() - 10

                # Clamp position
                constrained_x = max(min_x, min(new_pos.x(), max_x))
                constrained_y = max(min_y, min(new_pos.y(), max_y))

                value = QPointF(constrained_x, constrained_y)

            # Update all connections when the container moves
            for connection in self.connections:
                connection.update_position()

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
            used_outcomes.add(connection.outcome)
        return used_outcomes

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the rectangle edge closest to target."""
        center = self.scenePos()
        rect = self.rect()

        # Calculate angle to target
        angle = math.atan2(target_pos.y() - center.y(), target_pos.x() - center.x())

        # Rectangle dimensions
        w = rect.width() / 2
        h = rect.height() / 2

        # Determine which edge
        abs_tan = abs(math.tan(angle)) if math.cos(angle) != 0 else float("inf")
        if abs_tan <= h / w:
            # Left or right edge
            x = center.x() + w * (1 if math.cos(angle) > 0 else -1)
            y = center.y() + w * math.tan(angle) * (1 if math.cos(angle) > 0 else -1)
        else:
            # Top or bottom edge
            y = center.y() + h * (1 if math.sin(angle) > 0 else -1)
            x = (
                center.x() + h / math.tan(angle) * (1 if math.sin(angle) > 0 else -1)
                if math.sin(angle) != 0
                else center.x()
            )
        return QPointF(x, y)
