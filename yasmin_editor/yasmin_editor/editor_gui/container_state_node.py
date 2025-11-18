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
    """Container for State Machines and Concurrence states.
    
    Provides a graphical representation and management of hierarchical
    state containers including State Machines and Concurrence states.
    
    Attributes:
        name: Name of the container state.
        plugin_info: Plugin information (optional).
        is_state_machine: True if this is a State Machine container.
        is_concurrence: True if this is a Concurrence container.
        connections: List of outgoing connection lines.
        remappings: Dictionary of key remappings.
        final_outcomes: Dictionary of final outcome nodes.
        start_state: Initial state name for State Machines.
        default_outcome: Default outcome for Concurrence states.
        child_states: Dictionary of child state nodes.
        parent_container: Parent container if nested.
        min_width: Minimum width of the container.
        min_height: Minimum height of the container.
        xml_file: Path to XML file if loaded from file.
    """

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
        """Initialize a container state node.
        
        Args:
            name: Name of the container state.
            x: X coordinate for initial position.
            y: Y coordinate for initial position.
            is_concurrence: True for Concurrence, False for State Machine.
            remappings: Dictionary of key remappings.
            outcomes: List of outcome names.
            start_state: Initial state for State Machines.
            default_outcome: Default outcome for Concurrence states.
        """
        super().__init__(-500, -400, 1000, 800)
        self.name: str = name
        self.plugin_info: Optional["PluginInfo"] = None
        self.is_state_machine: bool = not is_concurrence
        self.is_concurrence: bool = is_concurrence
        self.connections: List["ConnectionLine"] = []
        self.remappings: Dict[str, str] = remappings or {}
        self.final_outcomes: Dict[str, "FinalOutcomeNode"] = {}
        self.start_state: Optional[str] = start_state
        self.default_outcome: Optional[str] = default_outcome
        self.child_states: Dict[
            str, Union["StateNode", "ContainerStateNode"]
        ] = {}
        self.parent_container: Optional["ContainerStateNode"] = None
        self.min_width: int = 1000
        self.min_height: int = 800
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
        self.header.setRect(-500, -400, 1000, 50)
        if is_concurrence:
            self.header.setBrush(QBrush(QColor(255, 140, 0)))
        else:
            self.header.setBrush(QBrush(QColor(0, 100, 200)))
        self.header.setPen(QPen(Qt.NoPen))

        self.title: QGraphicsTextItem = QGraphicsTextItem(self)
        self.title.setDefaultTextColor(Qt.white)
        title_font: QFont = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        self.title.setFont(title_font)

        type_label: str = "CONCURRENCE" if is_concurrence else "STATE MACHINE"
        self.title.setPlainText(f"{type_label}: {name}")
        title_rect = self.title.boundingRect()
        self.title.setPos(-title_rect.width() / 2, -385)

        if not is_concurrence:
            self.start_state_label: QGraphicsTextItem = (
                QGraphicsTextItem(self)
            )
            self.start_state_label.setDefaultTextColor(QColor(0, 100, 0))
            label_font: QFont = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.start_state_label.setFont(label_font)
            self.update_start_state_label()
        else:
            self.default_outcome_label: QGraphicsTextItem = (
                QGraphicsTextItem(self)
            )
            self.default_outcome_label.setDefaultTextColor(
                QColor(139, 69, 19)
            )
            label_font = QFont()
            label_font.setPointSize(8)
            label_font.setBold(True)
            self.default_outcome_label.setFont(label_font)
            self.update_default_outcome_label()

        self.connection_port: ConnectionPort = ConnectionPort(self)

    def update_start_state_label(self) -> None:
        """Update the initial state label text.
        
        Updates the visual label showing the start state for State Machine containers.
        """
        if not hasattr(self, "start_state_label"):
            return
        text = f"Initial: {self.start_state}" if self.start_state else (
            "Initial: (none)"
        )
        self.start_state_label.setPlainText(text)
        label_rect = self.start_state_label.boundingRect()
        self.start_state_label.setPos(-label_rect.width() / 2, -45)

    def update_default_outcome_label(self) -> None:
        """Update the default outcome label text for Concurrence.
        
        Updates the visual label showing the default outcome for Concurrence containers.
        """
        if not hasattr(self, "default_outcome_label"):
            return
        text = f"Default: {self.default_outcome}" if self.default_outcome else (
            "Default: (none)"
        )
        self.default_outcome_label.setPlainText(text)
        label_rect = self.default_outcome_label.boundingRect()
        self.default_outcome_label.setPos(-label_rect.width() / 2, -45)

    def update_visual_elements(self) -> None:
        """Update positions of header, title, and labels.
        
        Repositions all visual elements (header, title, labels, connection port)
        to match the current container dimensions.
        """
        rect = self.rect()

        self.header.setRect(rect.left(), rect.top(), rect.width(), 50)

        title_rect = self.title.boundingRect()
        self.title.setPos(
            rect.left() + rect.width() / 2 - title_rect.width() / 2,
            rect.top() + 12,
        )

        if hasattr(self, "start_state_label"):
            label_rect = self.start_state_label.boundingRect()
            self.start_state_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2,
                rect.top() + 52,
            )

        if hasattr(self, "default_outcome_label"):
            label_rect = self.default_outcome_label.boundingRect()
            self.default_outcome_label.setPos(
                rect.left() + rect.width() / 2 - label_rect.width() / 2,
                rect.top() + 52,
            )

        if hasattr(self, "connection_port"):
            self.connection_port.update_position_for_container()

    def add_child_state(
        self, state_node: Union["StateNode", "ContainerStateNode"]
    ) -> None:
        """Add a child state to this container.
        
        Args:
            state_node: The state node to add as a child.
        """
        if state_node.name not in self.child_states:
            self.child_states[state_node.name] = state_node
            state_node.parent_container = self
            state_node.setParentItem(self)
            self._layout_children()
            self.auto_resize_for_children()

    def remove_child_state(self, state_name: str) -> None:
        """Remove a child state from this container.
        
        Args:
            state_name: Name of the child state to remove.
        """
        if state_name in self.child_states:
            state: Union[
                "StateNode", "ContainerStateNode"
            ] = self.child_states[state_name]
            state.parent_container = None
            state.setParentItem(None)
            del self.child_states[state_name]
            self._layout_children()
            self.auto_resize_for_children()

    def _layout_children(self) -> None:
        """Layout child states in a deterministic grid pattern.
        
        Positions child states in a grid layout with configurable spacing
        and padding for visual clarity.
        """
        if not self.child_states:
            return

        rect = self.rect()
        CHILD_PADDING_X: int = 60
        CHILD_PADDING_Y: int = 110
        CHILD_SPACING_X: int = 700
        CHILD_SPACING_Y: int = 650
        CHILDREN_PER_COLUMN: int = 3

        sorted_children = sorted(
            self.child_states.items(), key=lambda x: x[0]
        )

        x_position: float = rect.left() + CHILD_PADDING_X
        y_position: float = rect.top() + CHILD_PADDING_Y
        column_index: int = 0
        row_index: int = 0

        for name, child in sorted_children:
            child.setPos(x_position, y_position)
            row_index += 1

            if row_index >= CHILDREN_PER_COLUMN:
                column_index += 1
                row_index = 0
                x_position += CHILD_SPACING_X
                y_position = rect.top() + CHILD_PADDING_Y
            else:
                if isinstance(child, ContainerStateNode):
                    child_height: float = child.rect().height()
                else:
                    child_height = child.boundingRect().height()
                y_position += max(child_height, 200) + CHILD_SPACING_Y

    def add_final_outcome(self, outcome_node: "FinalOutcomeNode") -> None:
        """Add a final outcome to this container.
        
        Args:
            outcome_node: The final outcome node to add.
        """
        if outcome_node.name not in self.final_outcomes:
            self.final_outcomes[outcome_node.name] = outcome_node
            outcome_node.parent_container = self
            outcome_node.setParentItem(self)

            self._reposition_final_outcomes()
            self.auto_resize_for_children()

    def _reposition_final_outcomes(self) -> None:
        """Reposition final outcomes on the right side of container.
        
        Places final outcome nodes vertically along the right edge of the
        container with appropriate spacing.
        """
        if not self.final_outcomes:
            return

        rect = self.rect()
        OUTCOME_PADDING_TOP: int = 110
        OUTCOME_SPACING_Y: int = 300

        max_child_x: float = rect.left() + 500

        for child in self.child_states.values():
            if isinstance(child, ContainerStateNode):
                child.prepareGeometryChange()
                child_right: float = child.pos().x() + child.rect().width()
            else:
                child_right = child.pos().x() + child.boundingRect().width()
            max_child_x = max(max_child_x, child_right)

        outcome_x: float = max_child_x + 300
        current_y: float = rect.top() + OUTCOME_PADDING_TOP

        sorted_outcomes = sorted(
            self.final_outcomes.items(), key=lambda x: x[0]
        )

        for name, outcome_node in sorted_outcomes:
            outcome_node.setPos(outcome_x, current_y)
            current_y += OUTCOME_SPACING_Y

    def auto_resize_for_children(self) -> None:
        """Automatically resize container to fit all children with padding.
        
        Calculates the minimum bounding box needed to contain all child states
        and final outcomes, then resizes the container accordingly.
        """
        if not self.child_states and not self.final_outcomes:
            return

        self.prepareGeometryChange()

        PADDING_LEFT: int = 50
        PADDING_RIGHT: int = 50
        PADDING_TOP: int = 100
        PADDING_BOTTOM: int = 50

        min_x: float = float("inf")
        max_x: float = float("-inf")
        min_y: float = float("inf")
        max_y: float = float("-inf")

        for child in self.child_states.values():
            child_pos = child.pos()
            if isinstance(child, ContainerStateNode):
                child_rect = child.rect()
                child_left = child_pos.x() + child_rect.left()
                child_right = child_pos.x() + child_rect.right()
                child_top = child_pos.y() + child_rect.top()
                child_bottom = child_pos.y() + child_rect.bottom()
            else:
                child_bounds = child.boundingRect()
                child_left = child_pos.x()
                child_right = child_pos.x() + child_bounds.width()
                child_top = child_pos.y()
                child_bottom = child_pos.y() + child_bounds.height()

            min_x = min(min_x, child_left)
            max_x = max(max_x, child_right)
            min_y = min(min_y, child_top)
            max_y = max(max_y, child_bottom)

        for outcome in self.final_outcomes.values():
            outcome_pos = outcome.pos()
            outcome_bounds = outcome.boundingRect()
            outcome_left = outcome_pos.x()
            outcome_right = outcome_pos.x() + outcome_bounds.width()
            outcome_top = outcome_pos.y()
            outcome_bottom = outcome_pos.y() + outcome_bounds.height()

            min_x = min(min_x, outcome_left)
            max_x = max(max_x, outcome_right)
            min_y = min(min_y, outcome_top)
            max_y = max(max_y, outcome_bottom)

        if min_x == float("inf"):
            new_width = self.min_width
            new_height = self.min_height
            new_left = -new_width / 2
            new_top = -new_height / 2
        else:
            new_left = min_x - PADDING_LEFT
            new_top = min_y - PADDING_TOP
            new_width = max(
                (max_x - min_x) + PADDING_LEFT + PADDING_RIGHT, self.min_width
            )
            new_height = max(
                (max_y - min_y) + PADDING_TOP + PADDING_BOTTOM,
                self.min_height,
            )

        self.setRect(new_left, new_top, new_width, new_height)

        self.update_visual_elements()

        self._update_all_connections_recursive()

        if self.parent_container:
            self.parent_container.auto_resize_for_children()

    def _update_all_connections_recursive(self) -> None:
        """Recursively update all connections in this container and children."""
        for connection in self.connections:
            connection.update_position()

        self.update_child_connections()

        for child in self.child_states.values():
            if isinstance(child, ContainerStateNode):
                child._update_all_connections_recursive()

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Handle double-click to edit container state.
        
        Args:
            event: The mouse event.
        """
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
        """Handle right-click context menu.
        
        Args:
            event: The context menu event.
        """
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

                action = menu.exec_(event.screenPos())

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
        """Update all connections for child states and outcomes recursively."""
        for child in self.child_states.values():
            for connection in child.connections:
                connection.update_position()

            if isinstance(child, ContainerStateNode):
                child.update_child_connections()

        for outcome in self.final_outcomes.values():
            for connection in outcome.connections:
                connection.update_position()

    def itemChange(
        self, change: QGraphicsItem.GraphicsItemChange, value: Any
    ) -> Any:
        """Handle item changes like position and selection.
        
        Args:
            change: The type of change occurring.
            value: The new value for the change.
            
        Returns:
            The potentially modified value.
        """
        if change == QGraphicsItem.ItemPositionChange and isinstance(
            value, QPointF
        ):
            if self.parent_container:
                container_rect = self.parent_container.rect()
                self_rect = self.rect()
                new_pos = value

                min_x = container_rect.left() - self_rect.left() + 10
                max_x = container_rect.right() - self_rect.right() - 10
                min_y = container_rect.top() - self_rect.top() + 40
                max_y = container_rect.bottom() - self_rect.bottom() - 10

                constrained_x = max(min_x, min(new_pos.x(), max_x))
                constrained_y = max(min_y, min(new_pos.y(), max_y))

                value = QPointF(constrained_x, constrained_y)

            for connection in self.connections:
                connection.update_position()

            self.update_child_connections()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            if self.parent_container:
                self.parent_container.auto_resize_for_children()

        elif change == QGraphicsItem.ItemSelectedChange:
            if value:
                self.setPen(QPen(QColor(255, 200, 0), 4))
            else:
                color = (
                    QColor(255, 140, 0)
                    if self.is_concurrence
                    else QColor(0, 0, 180)
                )
                self.setPen(QPen(color, 3))

        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine") -> None:
        """Add a connection line to this container.
        
        Args:
            connection: The connection line to add.
        """
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine") -> None:
        """Remove a connection line from this container.
        
        Args:
            connection: The connection line to remove.
        """
        if connection in self.connections:
            self.connections.remove(connection)

    def get_used_outcomes(self) -> Set[str]:
        """Get set of outcomes that already have connections.
        
        Returns:
            Set of outcome names that are already connected.
        """
        used_outcomes: Set[str] = set()
        for connection in self.connections:
            used_outcomes.add(connection.outcome)
        return used_outcomes

    def get_connection_point(self) -> QPointF:
        """Get the connection point for outgoing connections.
        
        Returns:
            Scene position of the connection port.
        """
        if hasattr(self, "connection_port"):
            return self.connection_port.scenePos()
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the intersection point on container edge towards target.
        
        Args:
            target_pos: The target position to point towards.
            
        Returns:
            Point on the container's edge closest to the target.
        """
        rect = self.rect()
        pos = self.scenePos()

        center_x = pos.x() + rect.left() + rect.width() / 2
        center_y = pos.y() + rect.top() + rect.height() / 2
        center = QPointF(center_x, center_y)

        angle = math.atan2(
            target_pos.y() - center.y(), target_pos.x() - center.x()
        )

        w = rect.width() / 2
        h = rect.height() / 2

        abs_tan = (
            abs(math.tan(angle))
            if math.cos(angle) != 0
            else float("inf")
        )
        if abs_tan <= h / w:
            x = center.x() + w * (1 if math.cos(angle) > 0 else -1)
            y = center.y() + w * math.tan(angle) * (
                1 if math.cos(angle) > 0 else -1
            )
        else:
            y = center.y() + h * (1 if math.sin(angle) > 0 else -1)
            x = (
                center.x()
                + h / math.tan(angle) * (1 if math.sin(angle) > 0 else -1)
                if math.sin(angle) != 0
                else center.x()
            )
        return QPointF(x, y)
