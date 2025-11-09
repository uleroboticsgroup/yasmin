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
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer
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
        start_state: str = None,
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
        self.start_state = start_state
        self.default_outcome = default_outcome  # For Concurrence
        self.child_states = {}  # Dict[str, Union[StateNode, ContainerStateNode]]
        self.parent_container = None
        self.min_width = 300
        self.min_height = 200
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
            self.start_state_label = QGraphicsTextItem(self)
            self.start_state_label.setDefaultTextColor(QColor(0, 100, 0))
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.start_state_label.setFont(label_font)
            self.update_start_state_label()
        else:
            # Add default outcome label (for Concurrence only)
            self.default_outcome_label = QGraphicsTextItem(self)
            self.default_outcome_label.setDefaultTextColor(QColor(139, 69, 19))  # Brown
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.default_outcome_label.setFont(label_font)
            self.update_default_outcome_label()

        # Add connection port for receiving connections from outside
        self.connection_port = ConnectionPort(self)

    def update_start_state_label(self):
        """Update the initial state label text."""
        if not hasattr(self, "start_state_label"):
            return
        if self.start_state:
            self.start_state_label.setPlainText(f"Initial: {self.start_state}")
        else:
            self.start_state_label.setPlainText("Initial: (none)")
        label_rect = self.start_state_label.boundingRect()
        self.start_state_label.setPos(-label_rect.width() / 2, -45)

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

    def update_visual_elements(self):
        """Update positions of header, title, and labels based on current rect."""
        rect = self.rect()

        # Update header
        self.header.setRect(rect.left(), rect.top(), rect.width(), 30)

        # Update title position
        title_rect = self.title.boundingRect()
        self.title.setPos(
            rect.left() + rect.width() / 2 - title_rect.width() / 2, rect.top() + 5
        )

        # Update initial state label position (for State Machine)
        if hasattr(self, "start_state_label"):
            label_rect = self.start_state_label.boundingRect()
            self.start_state_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2, rect.top() + 30
            )

        # Update default outcome label position (for Concurrence)
        if hasattr(self, "default_outcome_label"):
            label_rect = self.default_outcome_label.boundingRect()
            self.default_outcome_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2, rect.top() + 30
            )

        # Update connection port position
        if hasattr(self, "connection_port"):
            self.connection_port.update_position_for_container()

    def add_child_state(self, state_node):
        """Add a child state to this container."""
        if state_node.name not in self.child_states:
            self.child_states[state_node.name] = state_node
            state_node.parent_container = self
            state_node.setParentItem(self)

            # Position relative to container - place new states in available space
            rect = self.rect()
            # Start positioning from top-left, below header
            base_x = rect.left() + 80
            base_y = rect.top() + 70

            # Arrange in a grid
            index = len(self.child_states) - 1
            col = index % 3  # 3 columns
            row = index // 3
            state_node.setPos(base_x + col * 150, base_y + row * 120)

            self.auto_resize_for_children()

    def remove_child_state(self, state_name: str):
        """Remove a child state from this container."""
        if state_name in self.child_states:
            state = self.child_states[state_name]
            state.parent_container = None
            state.setParentItem(None)
            del self.child_states[state_name]
            self.auto_resize_for_children()

    def add_final_outcome(self, outcome_node):
        """Add a final outcome to this container."""
        if outcome_node.name not in self.final_outcomes:
            self.final_outcomes[outcome_node.name] = outcome_node
            outcome_node.parent_container = self
            outcome_node.setParentItem(self)

            # Position relative to container - place on the right side
            rect = self.rect()
            base_x = rect.right() - 100
            base_y = rect.top() + 70

            # Stack vertically
            index = len(self.final_outcomes) - 1
            outcome_node.setPos(base_x, base_y + index * 80)

            self.auto_resize_for_children()

    def auto_resize_for_children(self):
        """Automatically resize container to fit all children with padding."""
        if not self.child_states and not self.final_outcomes:
            # Reset to minimum size if empty
            rect = self.rect()
            self.setRect(rect.left(), rect.top(), self.min_width, self.min_height)
            self.update_visual_elements()
            # Trigger parent resize if this container is nested
            if self.parent_container:
                self.parent_container.auto_resize_for_children()
            return

        # Calculate bounding box of all children (states and outcomes)
        min_x, min_y = float("inf"), float("inf")
        max_x, max_y = float("-inf"), float("-inf")

        # Include child states
        for child in self.child_states.values():
            child_rect = child.boundingRect()
            child_pos = child.pos()

            # Get the actual bounds of the child
            # For containers, use their full rect size
            if isinstance(child, ContainerStateNode):
                child_rect = child.rect()

            left = child_pos.x() + child_rect.left()
            top = child_pos.y() + child_rect.top()
            right = child_pos.x() + child_rect.right()
            bottom = child_pos.y() + child_rect.bottom()

            min_x = min(min_x, left)
            min_y = min(min_y, top)
            max_x = max(max_x, right)
            max_y = max(max_y, bottom)

        # Include final outcomes
        for outcome in self.final_outcomes.values():
            outcome_rect = outcome.boundingRect()
            outcome_pos = outcome.pos()
            left = outcome_pos.x() + outcome_rect.left()
            top = outcome_pos.y() + outcome_rect.top()
            right = outcome_pos.x() + outcome_rect.right()
            bottom = outcome_pos.y() + outcome_rect.bottom()
            min_x = min(min_x, left)
            min_y = min(min_y, top)
            max_x = max(max_x, right)
            max_y = max(max_y, bottom)

        # Get current rect
        rect = self.rect()

        # Add padding - more padding for better spacing
        padding_left = 40
        padding_right = 40
        padding_top = 80  # Extra for header and initial state label
        padding_bottom = 40

        min_x -= padding_left
        min_y -= padding_top
        max_x += padding_right
        max_y += padding_bottom

        # Calculate new dimensions
        new_width = max(self.min_width, max_x - min_x)
        new_height = max(self.min_height, max_y - min_y)

        # Get current dimensions
        current_width = rect.width()
        current_height = rect.height()

        # Check if we need to resize
        needs_resize = new_width > current_width or new_height > current_height

        if needs_resize:
            # Notify the scene about the geometry change
            self.prepareGeometryChange()

            # Expand to fit children
            self.setRect(
                rect.left(),
                rect.top(),
                max(new_width, current_width),
                max(new_height, current_height),
            )

            # Update visual elements (including connection port position)
            self.update_visual_elements()

            # Recursively trigger parent resize if this container is nested
            # Do this BEFORE updating connections so all containers are resized first
            if self.parent_container:
                self.parent_container.auto_resize_for_children()
            else:
                # Only update connections at the root level (after all nested resizing is done)
                self._update_all_connections_recursive()

    def _update_all_connections_recursive(self):
        """Recursively update all connections for this container and its children."""
        # Update connections to/from this container
        for connection in self.connections:
            connection.update_position()

        # Update child connections recursively
        self.update_child_connections()

        # Update connections for nested containers
        for child in self.child_states.values():
            if isinstance(child, ContainerStateNode):
                child._update_all_connections_recursive()

    def update_size(self):
        """Legacy method - now calls auto_resize_for_children."""
        self.auto_resize_for_children()

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

    def contextMenuEvent(self, event):
        """Handle right-click context menu."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                from PyQt5.QtWidgets import QMenu

                menu = QMenu()

                # Add actions for containers
                add_state_action = menu.addAction("Add State")
                add_sm_action = menu.addAction("Add State Machine")
                add_cc_action = menu.addAction("Add Concurrence")
                menu.addSeparator()
                add_outcome_action = menu.addAction("Add Final Outcome")
                menu.addSeparator()
                edit_action = menu.addAction("Edit Properties")
                delete_action = menu.addAction("Delete")

                # Show menu and get action
                action = menu.exec_(event.screenPos())

                # Handle actions
                if action == add_state_action:
                    self.setSelected(True)
                    canvas.editor_ref.add_state_to_container()
                elif action == add_sm_action:
                    self.setSelected(True)
                    canvas.editor_ref.add_state_machine_to_container()
                elif action == add_cc_action:
                    self.setSelected(True)
                    canvas.editor_ref.add_concurrence_to_container()
                elif action == add_outcome_action:
                    self.setSelected(True)
                    canvas.editor_ref.add_final_outcome()
                elif action == edit_action:
                    self.setSelected(True)
                    canvas.editor_ref.edit_state()
                elif action == delete_action:
                    self.setSelected(True)
                    canvas.editor_ref.delete_selected()

                event.accept()
                return
        super().contextMenuEvent(event)

    def update_child_connections(self):
        """Recursively update all connections of child states and nested containers."""
        # Update connections of all child states
        for child in self.child_states.values():
            # Update child's own connections
            for connection in child.connections:
                connection.update_position()

            # If child is also a container, recursively update its children
            if isinstance(child, ContainerStateNode):
                child.update_child_connections()

        # Update connections of final outcomes
        for outcome in self.final_outcomes.values():
            for connection in outcome.connections:
                connection.update_position()

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

            # Update all child connections recursively
            self.update_child_connections()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            # After position has changed, trigger parent resize if nested
            if self.parent_container:
                self.parent_container.auto_resize_for_children()

        elif change == QGraphicsItem.ItemSelectedChange:
            # Highlight selected items in yellow
            if value:  # Selected
                if self.is_concurrence:
                    self.setPen(QPen(QColor(255, 200, 0), 4))  # Yellow highlight
                else:
                    self.setPen(QPen(QColor(255, 200, 0), 4))  # Yellow highlight
            else:  # Deselected
                if self.is_concurrence:
                    self.setPen(QPen(QColor(255, 140, 0), 3))  # Original dark orange
                else:
                    self.setPen(QPen(QColor(0, 0, 180), 3))  # Original dark blue

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
        """Get the point where connections should attach (connection port position)."""
        if hasattr(self, "connection_port"):
            return self.connection_port.scenePos()
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the rectangle edge closest to target."""
        center = self.scenePos()
        rect = self.rect()

        # Calculate angle to target
        angle = math.atan2(target_pos.y() - center.y(), target_pos.x() - center.x())

        # Rectangle dimensions (half-width and half-height)
        w = rect.width() / 2
        h = rect.height() / 2

        # Determine which edge the line intersects
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
