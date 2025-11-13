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

from typing import Optional, Union, TYPE_CHECKING
from PyQt5.QtWidgets import (
    QGraphicsView,
    QGraphicsScene,
    QGraphicsLineItem,
    QMenu,
    QWidget,
)
from PyQt5.QtCore import Qt, QLineF, QTimer, QPointF, QEvent
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter

from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class StateMachineCanvas(QGraphicsView):
    """Canvas for drawing and connecting state machine nodes."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QBrush(QColor(255, 255, 255)))

        self.is_dragging_connection: bool = False
        self.drag_start_node: Optional[
            Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
        ] = None
        self.temp_line: Optional[QGraphicsLineItem] = None
        self.editor_ref: Optional["YasminEditor"] = None

    def is_valid_connection(
        self,
        source_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        target_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
    ) -> bool:
        """Validate if a connection between two nodes is allowed."""
        if source_node == target_node:
            if isinstance(source_node, StateNode):
                source_container = getattr(source_node, "parent_container", None)
                if source_container and isinstance(source_container, ContainerStateNode):
                    if source_container.is_concurrence:
                        return True
            return False

        source_container: Optional["ContainerStateNode"] = getattr(
            source_node, "parent_container", None
        )
        target_container: Optional["ContainerStateNode"] = getattr(
            target_node, "parent_container", None
        )

        if isinstance(source_node, FinalOutcomeNode):
            if isinstance(target_node, ContainerStateNode):
                if source_container == target_node:
                    return True
                return True

            if isinstance(target_node, FinalOutcomeNode):
                if (
                    source_container
                    and target_container == source_container.parent_container
                ):
                    return True
                if source_container == target_container:
                    return True

            return source_container != target_container and target_container is None

        if isinstance(source_node, (StateNode, ContainerStateNode)):
            if isinstance(target_node, FinalOutcomeNode):
                return source_container == target_container

            if isinstance(target_node, (StateNode, ContainerStateNode)):
                return source_container == target_container

        return False

    def keyPressEvent(self, event: QEvent) -> None:
        """Handle keyboard shortcuts."""
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace):
            if self.editor_ref:
                self.editor_ref.delete_selected()
                event.accept()
                return
        super().keyPressEvent(event)

    def wheelEvent(self, event: QEvent) -> None:
        factor: float = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        self.scale(factor, factor)

    def start_connection_drag(
        self,
        from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        event: QEvent,
    ) -> None:
        """Start dragging a connection from a node's port."""
        self.drag_start_node = from_node
        self.is_dragging_connection = True
        self.setDragMode(QGraphicsView.NoDrag)

        self.temp_line = QGraphicsLineItem()
        pen: QPen = QPen(QColor(100, 100, 255), 3, Qt.DashLine)
        self.temp_line.setPen(pen)
        self.scene.addItem(self.temp_line)

        port_scene_pos: QPointF = from_node.connection_port.scenePos()
        self.temp_line.setLine(QLineF(port_scene_pos, port_scene_pos))

    def mousePressEvent(self, event: QEvent) -> None:
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QEvent) -> None:
        if self.is_dragging_connection and self.temp_line and self.drag_start_node:
            port_scene_pos: QPointF = self.drag_start_node.connection_port.scenePos()
            scene_pos: QPointF = self.mapToScene(event.pos())
            self.temp_line.setLine(QLineF(port_scene_pos, scene_pos))

            for scene_item in self.scene.items():
                if isinstance(
                    scene_item, (StateNode, FinalOutcomeNode, ContainerStateNode)
                ):
                    scene_item.setOpacity(1.0)

            item = self.itemAt(event.pos())
            target: Optional[
                Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
            ] = None
            if isinstance(item, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                target = item
            elif hasattr(item, "parentItem"):
                parent = item.parentItem()
                if isinstance(parent, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                    target = parent

            if (
                target
                and target != self.drag_start_node
                and self.is_valid_connection(self.drag_start_node, target)
            ):
                target.setOpacity(0.6)

            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QEvent) -> None:
        if event.button() == Qt.LeftButton and self.is_dragging_connection:
            scene_pos: QPointF = self.mapToScene(event.pos())
            items = self.scene.items(scene_pos)

            target_node: Optional[
                Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
            ] = None
            for item in items:
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

            if self.temp_line:
                self.scene.removeItem(self.temp_line)
                self.temp_line = None

            for scene_item in self.scene.items():
                if isinstance(
                    scene_item, (StateNode, FinalOutcomeNode, ContainerStateNode)
                ):
                    scene_item.setOpacity(1.0)

            source_node: Optional[
                Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
            ] = self.drag_start_node
            self.drag_start_node = None
            self.is_dragging_connection = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

            if (
                target_node
                and source_node
                and self.is_valid_connection(source_node, target_node)
            ):
                if self.editor_ref:
                    QTimer.singleShot(
                        0,
                        lambda src=source_node, tgt=target_node: self.editor_ref.create_connection_from_drag(
                            src, tgt
                        ),
                    )

            event.accept()
            return

        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event: QEvent) -> None:
        """Handle right-click context menu on canvas background."""
        item = self.itemAt(event.pos())
        if item is None:
            if self.editor_ref:
                menu: QMenu = QMenu()

                add_state_action = menu.addAction("Add State")
                add_sm_action = menu.addAction("Add State Machine")
                add_cc_action = menu.addAction("Add Concurrence")
                menu.addSeparator()
                add_outcome_action = menu.addAction("Add Final Outcome")

                action = menu.exec_(event.globalPos())

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
