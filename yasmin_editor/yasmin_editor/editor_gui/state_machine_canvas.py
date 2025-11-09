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

from PyQt5.QtWidgets import (
    QGraphicsView,
    QGraphicsScene,
    QGraphicsLineItem,
    QMenu,
)
from PyQt5.QtCore import Qt, QLineF, QTimer
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter

from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode


class StateMachineCanvas(QGraphicsView):
    """Canvas for drawing and connecting state machine nodes."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        # Set background
        self.setBackgroundBrush(QBrush(QColor(255, 255, 255)))  # White background

        # Connection dragging state
        self.is_dragging_connection = False
        self.drag_start_node = None
        self.temp_line = None

        # Reference to editor window
        self.editor_ref = None

    def is_valid_connection(self, source_node, target_node):
        """
        Validate if a connection between two nodes is allowed.

        Rules:
        - States, State Machines, and Concurrence can only connect within their own container
        - Final outcomes can receive connections from inside their container
        - Final outcomes can send connections outside their container (to parent's outcomes or external states)
        - Containers can receive connections from outside states and final outcomes
        - States within a Concurrence can connect to themselves (self-loop for repeater pattern)
        - No self-connections for other node types
        """
        # Allow self-connections ONLY for states inside a Concurrence
        if source_node == target_node:
            # Check if this is a state inside a Concurrence
            if isinstance(source_node, StateNode):
                source_container = getattr(source_node, "parent_container", None)
                if source_container and isinstance(source_container, ContainerStateNode):
                    if source_container.is_concurrence:
                        return True  # Allow self-loop in Concurrence
            return False  # Disallow for all other cases

        # Get parent containers
        source_container = getattr(source_node, "parent_container", None)
        target_container = getattr(target_node, "parent_container", None)

        # Source is a FinalOutcomeNode
        if isinstance(source_node, FinalOutcomeNode):
            # Final outcomes can connect to:
            # 1. Their own parent container (creates a loop back)
            # 2. Other containers at any level
            # 3. Final outcomes of parent container (one level up)
            # 4. States/containers outside their own container (root level)

            if isinstance(target_node, ContainerStateNode):
                # Can connect to any container, including their own parent
                # Check if target is the parent container (loop back)
                if source_container == target_node:
                    return True  # Allow loop back to parent container
                # Can connect to any other container
                return True

            if isinstance(target_node, FinalOutcomeNode):
                # Can connect to final outcomes of parent container
                # Check if target is in the parent of source's container
                if (
                    source_container
                    and target_container == source_container.parent_container
                ):
                    return True
                # Also allow if both are in the same container (for flexibility)
                if source_container == target_container:
                    return True

            # Can connect to states/containers outside their container
            return source_container != target_container and target_container is None

        # Source is a State or Container (SM/Concurrence)
        if isinstance(source_node, (StateNode, ContainerStateNode)):
            # Target is a FinalOutcomeNode
            if isinstance(target_node, FinalOutcomeNode):
                # Can only connect to final outcomes in the same container
                return source_container == target_container

            # Target is a State or Container
            if isinstance(target_node, (StateNode, ContainerStateNode)):
                # Must be in the same container (or both at root level)
                return source_container == target_container

        return False

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts."""
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace):
            if self.editor_ref:
                self.editor_ref.delete_selected()
                event.accept()
                return
        super().keyPressEvent(event)

    def show_context_menu(self, position):
        """Show context menu with options to add states, transitions, etc."""
        if not self.editor_ref:
            return

        menu = QMenu(self)

        # Add State action
        add_state_action = menu.addAction("Add State")
        add_state_action.triggered.connect(self.editor_ref.add_state)

        # Add State Machine action
        add_state_machine_action = menu.addAction("Add State Machine")
        add_state_machine_action.triggered.connect(self.editor_ref.add_state_machine)

        # Add Concurrence action
        add_concurrence_action = menu.addAction("Add Concurrence")
        add_concurrence_action.triggered.connect(self.editor_ref.add_concurrence)

        # Get selected items
        selected_items = self.scene.selectedItems()
        has_selected_state = any(
            isinstance(item, (StateNode, ContainerStateNode)) for item in selected_items
        )

        # Check if a user-created container is selected (not XML-based)
        has_selected_container = False
        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                # Check if it's a user-created container (not XML-based)
                if not hasattr(item, "xml_file") or item.xml_file is None:
                    has_selected_container = True
                    break

        # Add State to Container action (only if a user-created container is selected)
        if (
            has_selected_container
            and len(
                [item for item in selected_items if isinstance(item, ContainerStateNode)]
            )
            == 1
        ):
            add_to_container_action = menu.addAction("Add State to Container")
            add_to_container_action.triggered.connect(
                self.editor_ref.add_state_to_container
            )
            menu.addSeparator()

        # Edit State action (only if a single state is selected)
        if (
            has_selected_state
            and len(
                [
                    item
                    for item in selected_items
                    if isinstance(item, (StateNode, ContainerStateNode))
                ]
            )
            == 1
        ):
            edit_state_action = menu.addAction("Edit State")
            edit_state_action.triggered.connect(self.editor_ref.edit_state)

        # Add Transition action (only if a state is selected)
        add_transition_action = menu.addAction("Add Transition")
        add_transition_action.setEnabled(has_selected_state)
        add_transition_action.triggered.connect(self.editor_ref.add_transition)

        # Add Final Outcome action - changes text based on whether container is selected
        if has_selected_container:
            add_final_action = menu.addAction("Add Final Outcome to Container")
        else:
            add_final_action = menu.addAction("Add Final Outcome")
        add_final_action.triggered.connect(self.editor_ref.add_final_outcome)

        menu.addSeparator()

        # Delete Selected action (only if something is selected)
        delete_action = menu.addAction("Delete Selected")
        delete_action.setEnabled(len(selected_items) > 0)
        delete_action.triggered.connect(self.editor_ref.delete_selected)

        # Show menu at cursor position
        menu.exec_(self.mapToGlobal(position))

    def wheelEvent(self, event):
        # Zoom in/out
        factor = 1.2
        if event.angleDelta().y() < 0:
            factor = 1.0 / factor
        self.scale(factor, factor)

    def start_connection_drag(self, from_node, event):
        """Start dragging a connection from a node's port."""
        self.drag_start_node = from_node
        self.is_dragging_connection = True
        self.setDragMode(QGraphicsView.NoDrag)

        # Create temporary line
        self.temp_line = QGraphicsLineItem()
        pen = QPen(QColor(100, 100, 255), 3, Qt.DashLine)
        self.temp_line.setPen(pen)
        self.scene.addItem(self.temp_line)

        # Get the starting position (from the port's scene position)
        port_scene_pos = from_node.connection_port.scenePos()
        self.temp_line.setLine(QLineF(port_scene_pos, port_scene_pos))

    def mousePressEvent(self, event):
        # Check if we're starting a connection drag
        item = self.itemAt(event.pos())
        if isinstance(item, ConnectionPort):
            # The ConnectionPort will handle this
            pass

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.is_dragging_connection and self.temp_line and self.drag_start_node:
            # Update temporary line
            port_scene_pos = self.drag_start_node.connection_port.scenePos()
            scene_pos = self.mapToScene(event.pos())
            self.temp_line.setLine(QLineF(port_scene_pos, scene_pos))

            # Highlight potential target
            item = self.itemAt(event.pos())
            # Reset all opacities first
            for scene_item in self.scene.items():
                if isinstance(
                    scene_item, (StateNode, FinalOutcomeNode, ContainerStateNode)
                ):
                    scene_item.setOpacity(1.0)

            # Highlight if hovering over a valid target
            target = None
            if isinstance(item, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                target = item
            elif hasattr(item, "parentItem"):
                parent = item.parentItem()
                if isinstance(parent, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                    target = parent

            # Check if connection is valid
            if (
                target
                and target != self.drag_start_node
                and self.is_valid_connection(self.drag_start_node, target)
            ):
                target.setOpacity(0.6)

            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.is_dragging_connection:
            # Find target node using scene position
            scene_pos = self.mapToScene(event.pos())
            items = self.scene.items(scene_pos)

            target_node = None
            for item in items:
                # Check all node types including containers
                if isinstance(item, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                    target_node = item
                    break
                elif hasattr(item, "parentItem"):
                    parent = item.parentItem()
                    if isinstance(
                        parent, (StateNode, FinalOutcomeNode, ContainerStateNode)
                    ):
                        target_node = parent
                        break

            # Remove temporary line
            if self.temp_line:
                self.scene.removeItem(self.temp_line)
                self.temp_line = None

            # Reset opacity
            for scene_item in self.scene.items():
                if isinstance(
                    scene_item, (StateNode, FinalOutcomeNode, ContainerStateNode)
                ):
                    scene_item.setOpacity(1.0)

            # Create connection if valid target
            # Capture references before resetting drag state
            source_node = self.drag_start_node

            # Reset drag state first
            self.drag_start_node = None
            self.is_dragging_connection = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

            # Now create connection with captured references - validate first
            if (
                target_node
                and source_node
                and self.is_valid_connection(source_node, target_node)
            ):
                if self.editor_ref:
                    # Use QTimer to defer the connection creation
                    # This ensures the event loop processes properly
                    # Capture the nodes in the lambda's default arguments
                    QTimer.singleShot(
                        0,
                        lambda src=source_node, tgt=target_node: self.editor_ref.create_connection_from_drag(
                            src, tgt
                        ),
                    )

            event.accept()
            return

        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        """Handle right-click context menu on canvas background."""
        # Check if clicking on empty space (not on an item)
        item = self.itemAt(event.pos())
        if item is None:
            # Clicked on empty canvas
            if self.editor_ref:
                from PyQt5.QtWidgets import QMenu

                menu = QMenu()

                add_state_action = menu.addAction("Add State")
                add_sm_action = menu.addAction("Add State Machine")
                add_cc_action = menu.addAction("Add Concurrence")
                menu.addSeparator()
                add_outcome_action = menu.addAction("Add Final Outcome")

                # Show menu and get action
                action = menu.exec_(event.globalPos())

                # Handle actions
                if action == add_state_action:
                    self.editor_ref.add_state()
                elif action == add_sm_action:
                    self.editor_ref.add_state_machine()
                elif action == add_cc_action:
                    self.editor_ref.add_concurrence()
                elif action == add_outcome_action:
                    self.editor_ref.add_final_outcome()

                event.accept()
                return

        super().contextMenuEvent(event)
