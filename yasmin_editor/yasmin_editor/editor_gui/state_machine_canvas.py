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

from typing import TYPE_CHECKING, Optional, Union

from yasmin_editor.qt_compat import (
    Qt,
    QtCore,
    QtGui,
    QtWidgets,
    exec_menu,
    mouse_event_global_pos,
)

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class StateMachineCanvas(QtWidgets.QGraphicsView):
    """Canvas for drawing and connecting state machine nodes."""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.scene: QtWidgets.QGraphicsScene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.RubberBandDrag)
        self.setTransformationAnchor(
            QtWidgets.QGraphicsView.ViewportAnchor.AnchorUnderMouse
        )
        self.setResizeAnchor(QtWidgets.QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setBackgroundBrush(QtGui.QBrush(PALETTE.background))
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)

        self.is_dragging_connection: bool = False
        self.drag_start_node: Optional[
            Union["StateNode", "ContainerStateNode", "FinalOutcomeNode", "TextBlockNode"]
        ] = None
        self.temp_line: Optional[QtWidgets.QGraphicsLineItem] = None
        self.drag_rewire_connection: Optional["ConnectionLine"] = None
        self.editor_ref: Optional["YasminEditor"] = None
        self.pending_placement_item: Optional[
            Union["StateNode", "ContainerStateNode", "FinalOutcomeNode", "TextBlockNode"]
        ] = None
        self.pending_bundle_preview: Optional[QtWidgets.QGraphicsRectItem] = None
        self.pending_bundle_label: Optional[QtWidgets.QGraphicsSimpleTextItem] = None
        self._panning_with_middle_button = False
        self._last_pan_viewport_pos = QtCore.QPoint()
        self.layout_sync_handler = None

    def _schedule_layout_sync(self) -> None:
        if self.layout_sync_handler is not None:
            QtCore.QTimer.singleShot(0, self.layout_sync_handler)

    def get_preferred_placement_scene_pos(self) -> QtCore.QPointF:
        viewport_pos = self.viewport().mapFromGlobal(QtGui.QCursor.pos())
        if not self.viewport().rect().contains(viewport_pos):
            viewport_pos = self.viewport().rect().center()
        return self.mapToScene(viewport_pos)

    def start_pending_placement(
        self,
        item: Union[
            "StateNode", "ContainerStateNode", "FinalOutcomeNode", "TextBlockNode"
        ],
    ) -> None:
        self.pending_placement_item = item
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.NoDrag)
        self._update_pending_placement_item(self.get_preferred_placement_scene_pos())
        item.setSelected(True)
        if self.editor_ref:
            self.editor_ref.statusBar().showMessage(
                f"Place '{getattr(item, 'placement_label', getattr(item, 'name', 'item'))}' with left click. Right click or Escape cancels.",
                0,
            )

    def start_pending_bundle_placement(
        self, *, label: str, width: float, height: float
    ) -> None:
        self.clear_pending_bundle_placement()
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.NoDrag)
        preview = QtWidgets.QGraphicsRectItem(0.0, 0.0, float(width), float(height))
        preview.setBrush(QtGui.QBrush(PALETTE.ui_panel_alt_bg))
        preview.setPen(QtGui.QPen(PALETTE.selection_pen, 2, Qt.PenStyle.DashLine))
        preview.setOpacity(0.72)
        preview.setZValue(5000.0)
        preview.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.scene.addItem(preview)
        text_item = QtWidgets.QGraphicsSimpleTextItem(label, preview)
        text_item.setBrush(QtGui.QBrush(PALETTE.text_primary))
        text_rect = text_item.boundingRect()
        text_item.setPos(
            max(8.0, (float(width) - text_rect.width()) / 2.0),
            max(8.0, (float(height) - text_rect.height()) / 2.0),
        )
        self.pending_bundle_preview = preview
        self.pending_bundle_label = text_item
        self._update_pending_bundle_preview(self.get_preferred_placement_scene_pos())
        if self.editor_ref:
            self.editor_ref.statusBar().showMessage(
                f"{label}. Left click places it, right click or Escape cancels.", 0
            )

    def clear_pending_bundle_placement(self) -> None:
        if self.pending_bundle_preview is not None:
            self.scene.removeItem(self.pending_bundle_preview)
            self.pending_bundle_preview = None
            self.pending_bundle_label = None

    def clear_pending_placement(self) -> None:
        if self.pending_placement_item is not None:
            self.pending_placement_item = None
        self.clear_pending_bundle_placement()
        if not self.is_dragging_connection:
            self.setDragMode(QtWidgets.QGraphicsView.DragMode.RubberBandDrag)
        self._stop_middle_pan()

    def _update_pending_placement_item(self, scene_pos: QtCore.QPointF) -> None:
        if self.pending_placement_item is None:
            return
        self.pending_placement_item.setPos(scene_pos)

    def _update_pending_bundle_preview(self, scene_pos: QtCore.QPointF) -> None:
        if self.pending_bundle_preview is None:
            return
        self.pending_bundle_preview.setPos(scene_pos)

    def _start_middle_pan(self, viewport_pos: QtCore.QPoint) -> None:
        self._panning_with_middle_button = True
        self._last_pan_viewport_pos = QtCore.QPoint(viewport_pos)
        self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor)

    def _update_middle_pan(self, viewport_pos: QtCore.QPoint) -> None:
        if not self._panning_with_middle_button:
            return
        delta = viewport_pos - self._last_pan_viewport_pos
        self._last_pan_viewport_pos = QtCore.QPoint(viewport_pos)
        self.horizontalScrollBar().setValue(
            self.horizontalScrollBar().value() - delta.x()
        )
        self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())

    def _stop_middle_pan(self) -> None:
        self._panning_with_middle_button = False
        self.viewport().unsetCursor()

    def is_valid_connection(
        self,
        source_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        target_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
    ) -> bool:
        if self.editor_ref is None:
            return False
        current_model = self.editor_ref.current_container_model
        if isinstance(current_model, (Concurrence, OrthogonalState)):
            return not isinstance(source_node, FinalOutcomeNode) and isinstance(
                target_node, FinalOutcomeNode
            )
        if isinstance(source_node, FinalOutcomeNode):
            return False
        return True

    def _is_inline_text_editing_active(self) -> bool:
        focus_item = self.scene.focusItem()
        while focus_item is not None:
            if isinstance(focus_item, TextBlockNode):
                return bool(getattr(focus_item, "_is_editing", False))
            focus_item = focus_item.parentItem()
        for item in self.scene.selectedItems():
            if isinstance(item, TextBlockNode) and getattr(item, "_is_editing", False):
                return True
        return False

    def keyPressEvent(self, event: QtCore.QEvent) -> None:
        if event.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            if self._is_inline_text_editing_active():
                super().keyPressEvent(event)
                return
            if self.editor_ref:
                self.editor_ref.delete_selected()
                event.accept()
                return
        if event.key() == Qt.Key.Key_Escape:
            if self.pending_placement_item is not None:
                if self.editor_ref:
                    self.editor_ref.cancel_pending_node_placement(
                        self.pending_placement_item
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return
            if self.pending_bundle_preview is not None:
                if self.editor_ref:
                    self.editor_ref.cancel_pending_selection_placement()
                else:
                    self.clear_pending_placement()
                event.accept()
                return
        super().keyPressEvent(event)

    def wheelEvent(self, event: QtCore.QEvent) -> None:
        factor: float = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        self.scale(factor, factor)

    def _begin_connection_drag(
        self, from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
    ) -> None:
        self.drag_start_node = from_node
        self.is_dragging_connection = True
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.NoDrag)
        self.temp_line = QtWidgets.QGraphicsLineItem()
        pen: QtGui.QPen = QtGui.QPen(PALETTE.temp_connection, 3, Qt.PenStyle.DashLine)
        self.temp_line.setPen(pen)
        self.scene.addItem(self.temp_line)
        port_scene_pos: QtCore.QPointF = from_node.connection_port.scenePos()
        self.temp_line.setLine(QtCore.QLineF(port_scene_pos, port_scene_pos))

    def start_connection_drag(
        self,
        from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        event: QtCore.QEvent,
    ) -> None:
        if (
            self.pending_placement_item is not None
            or self.pending_bundle_preview is not None
        ):
            event.ignore()
            return
        if self.editor_ref and self.editor_ref.is_read_only_mode():
            event.ignore()
            return
        self.drag_rewire_connection = None
        self._begin_connection_drag(from_node)

    def start_rewire_drag(self, connection: "ConnectionLine") -> None:
        if (
            self.pending_placement_item is not None
            or self.pending_bundle_preview is not None
        ):
            return
        if self.editor_ref and self.editor_ref.is_read_only_mode():
            return
        self.drag_rewire_connection = connection
        self._begin_connection_drag(connection.from_node)
        if self.editor_ref:
            self.editor_ref.statusBar().showMessage(
                f"Rewiring transition '{connection.outcome}' from {connection.from_node.name}. Click a new target state."
            )

    def mousePressEvent(self, event: QtCore.QEvent) -> None:
        if self.pending_placement_item is not None:
            if event.button() == Qt.MouseButton.LeftButton:
                scene_pos = self.mapToScene(event.pos())
                if self.editor_ref:
                    self.editor_ref.finalize_pending_node_placement(
                        self.pending_placement_item, scene_pos
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return
            if event.button() == Qt.MouseButton.RightButton:
                if self.editor_ref:
                    self.editor_ref.cancel_pending_node_placement(
                        self.pending_placement_item
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return
        if self.pending_bundle_preview is not None:
            if event.button() == Qt.MouseButton.LeftButton:
                if self.editor_ref:
                    self.editor_ref.finalize_pending_selection_placement(
                        self.mapToScene(event.pos())
                    )
                else:
                    self.clear_pending_placement()
                event.accept()
                return
            if event.button() == Qt.MouseButton.RightButton:
                if self.editor_ref:
                    self.editor_ref.cancel_pending_selection_placement()
                else:
                    self.clear_pending_placement()
                event.accept()
                return
        if event.button() == Qt.MouseButton.BackButton and self.editor_ref:
            self.editor_ref.navigate_up_one_level()
            event.accept()
            return
        if event.button() == Qt.MouseButton.MiddleButton:
            self._start_middle_pan(event.pos())
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtCore.QEvent) -> None:
        if self.pending_placement_item is not None:
            self._update_pending_placement_item(self.mapToScene(event.pos()))
            event.accept()
            return
        if self.pending_bundle_preview is not None:
            self._update_pending_bundle_preview(self.mapToScene(event.pos()))
            event.accept()
            return
        if self._panning_with_middle_button:
            self._update_middle_pan(event.pos())
            event.accept()
            return
        if self.is_dragging_connection and self.temp_line and self.drag_start_node:
            port_scene_pos: QtCore.QPointF = (
                self.drag_start_node.connection_port.scenePos()
            )
            scene_pos: QtCore.QPointF = self.mapToScene(event.pos())
            self.temp_line.setLine(QtCore.QLineF(port_scene_pos, scene_pos))
            for scene_item in self.scene.items():
                if isinstance(
                    scene_item, (StateNode, FinalOutcomeNode, ContainerStateNode)
                ):
                    scene_item.setOpacity(1.0)
            item = self.itemAt(event.pos())
            target: Optional[
                Union[
                    "StateNode",
                    "ContainerStateNode",
                    "FinalOutcomeNode",
                    "TextBlockNode",
                ]
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

    def mouseReleaseEvent(self, event: QtCore.QEvent) -> None:
        if (
            event.button() == Qt.MouseButton.MiddleButton
            and self._panning_with_middle_button
        ):
            self._stop_middle_pan()
            event.accept()
            return
        if event.button() == Qt.MouseButton.LeftButton and self.is_dragging_connection:
            scene_pos: QtCore.QPointF = self.mapToScene(event.pos())
            items = self.scene.items(scene_pos)
            target_node: Optional[
                Union[
                    "StateNode",
                    "ContainerStateNode",
                    "FinalOutcomeNode",
                    "TextBlockNode",
                ]
            ] = None
            for item in items:
                if isinstance(item, (StateNode, FinalOutcomeNode, ContainerStateNode)):
                    target_node = item
                    break
                if hasattr(item, "parentItem"):
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
            self.setDragMode(QtWidgets.QGraphicsView.DragMode.RubberBandDrag)
            if (
                target_node
                and source_node
                and self.is_valid_connection(source_node, target_node)
            ):
                if self.editor_ref:
                    if rewire_connection is not None:
                        QtCore.QTimer.singleShot(
                            0,
                            lambda connection=rewire_connection, tgt=target_node: self.editor_ref.rewire_connection(
                                connection, tgt
                            ),
                        )
                    else:
                        QtCore.QTimer.singleShot(
                            0,
                            lambda src=source_node, tgt=target_node: self.editor_ref.create_connection_from_drag(
                                src, tgt
                            ),
                        )
            elif rewire_connection is not None and self.editor_ref:
                self.editor_ref.statusBar().showMessage("Rewiring canceled", 2000)
            event.accept()
            return
        super().mouseReleaseEvent(event)
        if (
            event.button() == Qt.MouseButton.LeftButton
            and not self._panning_with_middle_button
        ):
            self._schedule_layout_sync()
            if self.editor_ref is not None:
                self.editor_ref.handle_canvas_external_drop(
                    self, self.viewport().mapToGlobal(event.pos())
                )

    def contextMenuEvent(self, event: QtCore.QEvent) -> None:
        if (
            self.pending_placement_item is not None
            or self.pending_bundle_preview is not None
        ):
            event.accept()
            return
        item = self.itemAt(event.pos())
        if item is None and self.editor_ref:
            menu: QtWidgets.QMenu = QtWidgets.QMenu()
            if self.editor_ref.is_read_only_mode():
                view_container_action = menu.addAction("View Current Container")
                fit_action = menu.addAction("Fit")
                action = exec_menu(menu, mouse_event_global_pos(event))
                if action == view_container_action:
                    self.editor_ref.edit_current_container()
                elif action == fit_action:
                    self.editor_ref.fit_current_view()
                event.accept()
                return
            add_state_action = menu.addAction("Add State")
            add_sm_action = menu.addAction("Add State Machine")
            add_cc_action = menu.addAction("Add Concurrence")
            add_js_action = menu.addAction("Add Join State")
            menu.addSeparator()
            add_outcome_action = menu.addAction("Add Final Outcome")
            add_text_action = menu.addAction("Add Text")
            menu.addSeparator()
            edit_container_action = menu.addAction("Edit Current Container")
            action = exec_menu(menu, mouse_event_global_pos(event))
            if action == add_state_action:
                self.editor_ref.add_state()
            elif action == add_sm_action:
                self.editor_ref.add_state_machine()
            elif action == add_cc_action:
                self.editor_ref.add_concurrence()
            elif action == add_js_action:
                self.editor_ref.add_join_state()
            elif action == add_outcome_action:
                self.editor_ref.add_final_outcome()
            elif action == add_text_action:
                self.editor_ref.add_text_block()
            elif action == edit_container_action:
                self.editor_ref.edit_current_container()
            event.accept()
            return
        super().contextMenuEvent(event)
