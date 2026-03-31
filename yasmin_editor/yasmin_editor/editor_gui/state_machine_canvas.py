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

from typing import TYPE_CHECKING, Optional, Union

from PyQt5.QtCore import QEvent, QLineF, QPointF, Qt, QTimer
from PyQt5.QtGui import QBrush, QCursor, QPainter, QPen
from PyQt5.QtWidgets import (
    QGraphicsLineItem,
    QGraphicsScene,
    QGraphicsView,
    QMenu,
    QWidget,
)

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.model.concurrence import Concurrence

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class StateMachineCanvas(QGraphicsView):
    """Canvas for drawing and connecting state machine nodes."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QBrush(PALETTE.background))
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)

        self.is_dragging_connection: bool = False
        self.drag_start_node: Optional[
            Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
        ] = None
        self.temp_line: Optional[QGraphicsLineItem] = None
        self.drag_rewire_connection: Optional["ConnectionLine"] = None
        self.editor_ref: Optional["YasminEditor"] = None
        self.pending_placement_item: Optional[
            Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
        ] = None

    def get_preferred_placement_scene_pos(self) -> QPointF:
        viewport_pos = self.viewport().mapFromGlobal(QCursor.pos())
        if not self.viewport().rect().contains(viewport_pos):
            viewport_pos = self.viewport().rect().center()
        return self.mapToScene(viewport_pos)

    def start_pending_placement(
        self,
        item: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
    ) -> None:
        self.pending_placement_item = item
        self.setDragMode(QGraphicsView.NoDrag)
        self._update_pending_placement_item(self.get_preferred_placement_scene_pos())
        item.setSelected(True)
        if self.editor_ref:
            self.editor_ref.statusBar().showMessage(
                f"Place '{item.name}' with left click. Right click or Escape cancels.",
                0,
            )

    def clear_pending_placement(self) -> None:
        if self.pending_placement_item is not None:
            self.pending_placement_item = None
        if not self.is_dragging_connection:
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def _update_pending_placement_item(self, scene_pos: QPointF) -> None:
        if self.pending_placement_item is None:
            return
        self.pending_placement_item.setPos(scene_pos)

    def is_valid_connection(
        self,
        source_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        target_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
    ) -> bool:
        if self.editor_ref is None:
            return False

        current_model = self.editor_ref.current_container_model

        if isinstance(current_model, Concurrence):
            return not isinstance(source_node, FinalOutcomeNode) and isinstance(
                target_node, FinalOutcomeNode
            )

        if isinstance(source_node, FinalOutcomeNode):
            return False

        if source_node == target_node:
            return True

        return True

    def keyPressEvent(self, event: QEvent) -> None:
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace):
            if self.editor_ref:
                self.editor_ref.delete_selected()
                event.accept()
                return

        if event.key() == Qt.Key_Escape and self.pending_placement_item is not None:
            if self.editor_ref:
                self.editor_ref.cancel_pending_node_placement(self.pending_placement_item)
            else:
                self.clear_pending_placement()
            event.accept()
            return

        super().keyPressEvent(event)

    def wheelEvent(self, event: QEvent) -> None:
        factor: float = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        self.scale(factor, factor)

    def _begin_connection_drag(
        self,
        from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
    ) -> None:
        self.drag_start_node = from_node
        self.is_dragging_connection = True
        self.setDragMode(QGraphicsView.NoDrag)

        self.temp_line = QGraphicsLineItem()
        pen: QPen = QPen(PALETTE.temp_connection, 3, Qt.DashLine)
        self.temp_line.setPen(pen)
        self.scene.addItem(self.temp_line)

        port_scene_pos: QPointF = from_node.connection_port.scenePos()
        self.temp_line.setLine(QLineF(port_scene_pos, port_scene_pos))

    def start_connection_drag(
        self,
        from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        event: QEvent,
    ) -> None:
        if self.pending_placement_item is not None:
            event.ignore()
            return
        if self.editor_ref and self.editor_ref.is_read_only_mode():
            event.ignore()
            return
        self.drag_rewire_connection = None
        self._begin_connection_drag(from_node)

    def start_rewire_drag(self, connection: "ConnectionLine") -> None:
        if self.pending_placement_item is not None:
            return
        if self.editor_ref and self.editor_ref.is_read_only_mode():
            return
        self.drag_rewire_connection = connection
        self._begin_connection_drag(connection.from_node)
        if self.editor_ref:
            self.editor_ref.statusBar().showMessage(
                f"Rewiring transition '{connection.outcome}' from {connection.from_node.name}. Click a new target state."
            )

    def mousePressEvent(self, event: QEvent) -> None:
        if self.pending_placement_item is not None:
            if event.button() == Qt.LeftButton:
                scene_pos = self.mapToScene(event.pos())
                if self.editor_ref:
                    self.editor_ref.finalize_pending_node_placement(
                        self.pending_placement_item,
                        scene_pos,
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return

            if event.button() == Qt.RightButton:
                if self.editor_ref:
                    self.editor_ref.cancel_pending_node_placement(
                        self.pending_placement_item
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return

        if event.button() == Qt.BackButton and self.editor_ref:
            self.editor_ref.navigate_up_one_level()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QEvent) -> None:
        if self.pending_placement_item is not None:
            self._update_pending_placement_item(self.mapToScene(event.pos()))
            event.accept()
            return

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

            if target and self.is_valid_connection(self.drag_start_node, target):
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

            source_node = self.drag_start_node
            rewire_connection = self.drag_rewire_connection
            self.drag_start_node = None
            self.drag_rewire_connection = None
            self.is_dragging_connection = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

            if (
                target_node
                and source_node
                and self.is_valid_connection(source_node, target_node)
            ):
                if self.editor_ref:
                    if rewire_connection is not None:
                        QTimer.singleShot(
                            0,
                            lambda connection=rewire_connection, tgt=target_node: self.editor_ref.rewire_connection(
                                connection, tgt
                            ),
                        )
                    else:
                        QTimer.singleShot(
                            0,
                            lambda src=source_node, tgt=target_node: self.editor_ref.create_connection_from_drag(
                                src, tgt
                            ),
                        )
            elif rewire_connection is not None and self.editor_ref:
                self.editor_ref.statusBar().showMessage(
                    "Rewiring canceled",
                    2000,
                )

            event.accept()
            return

        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event: QEvent) -> None:
        if self.pending_placement_item is not None:
            event.accept()
            return

        item = self.itemAt(event.pos())
        if item is None and self.editor_ref:
            menu: QMenu = QMenu()
            if self.editor_ref.is_read_only_mode():
                view_container_action = menu.addAction("View Current Container")
                fit_action = menu.addAction("Fit")
                action = menu.exec_(event.globalPos())
                if action == view_container_action:
                    self.editor_ref.edit_current_container()
                elif action == fit_action:
                    self.editor_ref.fit_current_view()
                event.accept()
                return

            add_state_action = menu.addAction("Add State")
            add_sm_action = menu.addAction("Add State Machine")
            add_cc_action = menu.addAction("Add Concurrence")
            menu.addSeparator()
            add_outcome_action = menu.addAction("Add Final Outcome")
            menu.addSeparator()
            edit_container_action = menu.addAction("Edit Current Container")

            action = menu.exec_(event.globalPos())

            if action == add_state_action:
                self.editor_ref.add_state()
            elif action == add_sm_action:
                self.editor_ref.add_state_machine()
            elif action == add_cc_action:
                self.editor_ref.add_concurrence()
            elif action == add_outcome_action:
                self.editor_ref.add_final_outcome()
            elif action == edit_container_action:
                self.editor_ref.edit_current_container()

            event.accept()
            return

        super().contextMenuEvent(event)
