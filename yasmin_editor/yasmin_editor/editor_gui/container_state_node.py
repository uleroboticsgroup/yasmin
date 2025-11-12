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
from typing import Dict, List, Optional, Union, Any, Set, TYPE_CHECKING
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem, QGraphicsRectItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont

from yasmin_editor.editor_gui.connection_port import ConnectionPort

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.state_node import StateNode
    from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
    from yasmin_editor.plugins_manager.plugin_info import PluginInfo


class ContainerStateNode(QGraphicsRectItem):
    """Container for State Machines and Concurrence states that can hold child states."""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        is_concurrence: bool = False,
        remappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
        start_state: Optional[str] = None,
        default_outcome: Optional[str] = None,
    ) -> None:
        super().__init__(-200, -125, 400, 250)
        self.name: str = name
        self.plugin_info: Optional["PluginInfo"] = None
        self.is_state_machine: bool = not is_concurrence
        self.is_concurrence: bool = is_concurrence
        self.connections: List["ConnectionLine"] = []
        self.remappings: Dict[str, str] = remappings or {}
        self.final_outcomes: Dict[str, "FinalOutcomeNode"] = {}
        self.start_state: Optional[str] = start_state
        self.default_outcome: Optional[str] = default_outcome
        self.child_states: Dict[str, Union["StateNode", "ContainerStateNode"]] = {}
        self.parent_container: Optional["ContainerStateNode"] = None
        self.min_width: int = 400
        self.min_height: int = 250
        self.xml_file: Optional[str] = None

        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        if is_concurrence:
            self.setBrush(QBrush(QColor(255, 220, 150, 180)))
            self.setPen(QPen(QColor(255, 140, 0), 3))
        else:
            self.setBrush(QBrush(QColor(173, 216, 230, 180)))
            self.setPen(QPen(QColor(0, 0, 180), 3))

        self.header: QGraphicsRectItem = QGraphicsRectItem(self)
        self.header.setRect(-200, -125, 400, 35)
        if is_concurrence:
            self.header.setBrush(QBrush(QColor(255, 140, 0)))
        else:
            self.header.setBrush(QBrush(QColor(0, 100, 200)))
        self.header.setPen(QPen(Qt.NoPen))

        self.title: QGraphicsTextItem = QGraphicsTextItem(self)
        self.title.setDefaultTextColor(Qt.white)
        title_font: QFont = QFont()
        title_font.setPointSize(11)
        title_font.setBold(True)
        self.title.setFont(title_font)

        type_label: str = "CONCURRENCE" if is_concurrence else "STATE MACHINE"
        self.title.setPlainText(f"{type_label}: {name}")
        title_rect = self.title.boundingRect()
        self.title.setPos(-title_rect.width() / 2, -118)

        if not is_concurrence:
            self.start_state_label: QGraphicsTextItem = QGraphicsTextItem(self)
            self.start_state_label.setDefaultTextColor(QColor(0, 100, 0))
            label_font: QFont = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.start_state_label.setFont(label_font)
            self.update_start_state_label()
        else:
            self.default_outcome_label: QGraphicsTextItem = QGraphicsTextItem(self)
            self.default_outcome_label.setDefaultTextColor(QColor(139, 69, 19))
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.default_outcome_label.setFont(label_font)
            self.update_default_outcome_label()

        self.connection_port: ConnectionPort = ConnectionPort(self)

    def update_start_state_label(self) -> None:
        """Update the initial state label text."""
        if not hasattr(self, "start_state_label"):
            return
        if self.start_state:
            self.start_state_label.setPlainText(f"Initial: {self.start_state}")
        else:
            self.start_state_label.setPlainText("Initial: (none)")
        label_rect = self.start_state_label.boundingRect()
        self.start_state_label.setPos(-label_rect.width() / 2, -45)

    def update_default_outcome_label(self) -> None:
        """Update the default outcome label text for Concurrence."""
        if not hasattr(self, "default_outcome_label"):
            return
        if self.default_outcome:
            self.default_outcome_label.setPlainText(f"Default: {self.default_outcome}")
        else:
            self.default_outcome_label.setPlainText("Default: (none)")
        label_rect = self.default_outcome_label.boundingRect()
        self.default_outcome_label.setPos(-label_rect.width() / 2, -45)

    def update_visual_elements(self) -> None:
        """Update positions of header, title, and labels based on current rect."""
        rect = self.rect()

        # Update header - make it slightly taller for better visibility
        self.header.setRect(rect.left(), rect.top(), rect.width(), 35)

        # Update title position
        title_rect = self.title.boundingRect()
        self.title.setPos(
            rect.left() + rect.width() / 2 - title_rect.width() / 2, rect.top() + 7
        )

        # Update initial state label position (for State Machine)
        if hasattr(self, "start_state_label"):
            label_rect = self.start_state_label.boundingRect()
            self.start_state_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2, rect.top() + 35
            )

        # Update default outcome label position (for Concurrence)
        if hasattr(self, "default_outcome_label"):
            label_rect = self.default_outcome_label.boundingRect()
            self.default_outcome_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2, rect.top() + 35
            )

        # Update connection port position
        if hasattr(self, "connection_port"):
            self.connection_port.update_position_for_container()

    def add_child_state(
        self, state_node: Union["StateNode", "ContainerStateNode"]
    ) -> None:
        """Add a child state to this container."""
        if state_node.name not in self.child_states:
            self.child_states[state_node.name] = state_node
            state_node.parent_container = self
            state_node.setParentItem(self)

            rect = self.rect()
            CHILD_PADDING_X: int = 80
            CHILD_PADDING_Y: int = 100
            CHILD_SPACING_Y: int = 200

            y_position: float = rect.top() + CHILD_PADDING_Y

            for i, existing_child in enumerate(list(self.child_states.values())[:-1]):
                if isinstance(existing_child, ContainerStateNode):
                    child_height: float = existing_child.rect().height()
                else:
                    child_height = existing_child.boundingRect().height()
                y_position += child_height + CHILD_SPACING_Y

            x: float = rect.left() + CHILD_PADDING_X
            state_node.setPos(x, y_position)

            self.auto_resize_for_children()

    def remove_child_state(self, state_name: str) -> None:
        """Remove a child state from this container."""
        if state_name in self.child_states:
            state: Union["StateNode", "ContainerStateNode"] = self.child_states[
                state_name
            ]
            state.parent_container = None
            state.setParentItem(None)
            del self.child_states[state_name]
            self.auto_resize_for_children()

    def add_final_outcome(self, outcome_node: "FinalOutcomeNode") -> None:
        """Add a final outcome to this container."""
        if outcome_node.name not in self.final_outcomes:
            self.final_outcomes[outcome_node.name] = outcome_node
            outcome_node.parent_container = self
            outcome_node.setParentItem(self)

            # Re-position all final outcomes to ensure they're aligned properly
            self._reposition_final_outcomes()
            self.auto_resize_for_children()

    def _reposition_final_outcomes(self) -> None:
        """Reposition all final outcomes vertically on the right side of the container."""
        if not self.final_outcomes:
            return

        rect = self.rect()
        OUTCOME_PADDING_TOP: int = 100
        OUTCOME_SPACING_Y: int = 200

        max_child_x: float = rect.left() + 400

        for child in self.child_states.values():
            if isinstance(child, ContainerStateNode):
                child.prepareGeometryChange()
                child_right: float = child.pos().x() + child.rect().width()
            else:
                child_right = child.pos().x() + child.boundingRect().width()
            max_child_x = max(max_child_x, child_right)

        outcome_x: float = max_child_x + 250
        current_y: float = rect.top() + OUTCOME_PADDING_TOP

        for outcome_node in self.final_outcomes.values():
            outcome_node.setPos(outcome_x, current_y)
            current_y += OUTCOME_SPACING_Y

    def auto_resize_for_children(self) -> None:
        """Automatically resize container to fit all children with padding."""
        if not self.child_states and not self.final_outcomes:
            rect = self.rect()
            self.setRect(rect.left(), rect.top(), self.min_width, self.min_height)
            self.update_visual_elements()
            if self.parent_container:
                self.parent_container.auto_resize_for_children()
            return

        min_x, min_y = float("inf"), float("inf")
        max_x, max_y = float("-inf"), float("-inf")

        for child in self.child_states.values():
            child_rect = child.boundingRect()
            child_pos = child.pos()

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

        rect = self.rect()

        padding_left = 40
        padding_right = 60
        padding_top = 70
        padding_bottom = 50

        min_x -= padding_left
        min_y -= padding_top
        max_x += padding_right
        max_y += padding_bottom

        new_width = max(self.min_width, max_x - min_x)
        new_height = max(self.min_height, max_y - min_y)

        current_width = rect.width()
        current_height = rect.height()

        needs_resize = new_width > current_width or new_height > current_height

        if needs_resize:
            self.prepareGeometryChange()

            self.setRect(
                rect.left(),
                rect.top(),
                max(new_width, current_width),
                max(new_height, current_height),
            )

            self.update_visual_elements()

            if self.parent_container:
                self.parent_container.auto_resize_for_children()
            else:
                self._update_all_connections_recursive()

    def _update_all_connections_recursive(self) -> None:
        """Recursively update all connections for this container and its children."""
        for connection in self.connections:
            connection.update_position()

        self.update_child_connections()

        for child in self.child_states.values():
            if isinstance(child, ContainerStateNode):
                child._update_all_connections_recursive()

    def mouseDoubleClickEvent(self, event: Any) -> None:
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

    def contextMenuEvent(self, event: Any) -> None:
        """Handle right-click context menu."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                from PyQt5.QtWidgets import QMenu

                menu: QMenu = QMenu()

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

    def update_child_connections(self) -> None:
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

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
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
            # Highlight selected items with bright yellow/orange
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

    def add_connection(self, connection: "ConnectionLine") -> None:
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine") -> None:
        if connection in self.connections:
            self.connections.remove(connection)

    def get_used_outcomes(self) -> Set[str]:
        """Get set of outcomes that already have connections."""
        used_outcomes: Set[str] = set()
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
        # Get the rect and calculate the center in scene coordinates
        rect = self.rect()
        pos = self.scenePos()

        # Calculate the actual center of the rectangle in scene coordinates
        center_x = pos.x() + rect.left() + rect.width() / 2
        center_y = pos.y() + rect.top() + rect.height() / 2
        center = QPointF(center_x, center_y)

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
