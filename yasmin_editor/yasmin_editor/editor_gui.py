#!/usr/bin/env python3

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

import sys
import os
import math
from typing import Dict, List, Optional, Tuple
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QListWidget,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsItem,
    QGraphicsEllipseItem,
    QGraphicsLineItem,
    QGraphicsTextItem,
    QGraphicsRectItem,
    QLabel,
    QInputDialog,
    QMessageBox,
    QFileDialog,
    QSplitter,
    QListWidgetItem,
    QDialog,
    QFormLayout,
    QLineEdit,
    QComboBox,
    QDialogButtonBox,
    QAction,
    QToolBar,
    QGraphicsPolygonItem,
    QGraphicsPathItem,
    QMenu,
)
from PyQt5.QtCore import Qt, QPointF, QRectF, QLineF, pyqtSignal, QEvent, QTimer
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter, QFont, QPolygonF, QPainterPath

from yasmin_editor.plugin_manager import PluginManager
from yasmin_editor.plugin_info import PluginInfo


class ConnectionPort(QGraphicsEllipseItem):
    """Connection port for drag-to-connect functionality."""

    def __init__(self, parent_state):
        super().__init__(-5, -5, 10, 10, parent_state)
        self.parent_state = parent_state
        self.setBrush(QBrush(QColor(100, 100, 255)))
        self.setPen(QPen(QColor(0, 0, 100), 1))
        self.setPos(60, 0)  # Right edge
        self.setCursor(Qt.CrossCursor)

        # Make it accept mouse events but not movable
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)

    def mousePressEvent(self, event):
        # Don't propagate to parent - we'll handle this in the canvas
        if event.button() == Qt.LeftButton:
            # Find the canvas view
            if self.scene() and self.scene().views():
                canvas = self.scene().views()[0]
                if hasattr(canvas, "start_connection_drag"):
                    canvas.start_connection_drag(self.parent_state, event)
                    event.accept()
                    return
        event.ignore()

    def mouseMoveEvent(self, event):
        # Prevent parent from receiving move events
        event.accept()

    def mouseReleaseEvent(self, event):
        # Prevent parent from receiving release events
        event.accept()


class StateNode(QGraphicsEllipseItem):
    """Graphical representation of a state in the state machine."""

    def __init__(
        self,
        name: str,
        plugin_info: PluginInfo,
        x: float,
        y: float,
        is_state_machine: bool = False,
    ):
        super().__init__(-60, -40, 120, 80)
        self.name = name
        self.plugin_info = plugin_info
        self.is_state_machine = is_state_machine
        self.connections: List["ConnectionLine"] = []

        # Set position
        self.setPos(x, y)

        # Set flags
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        # Set colors based on type
        if is_state_machine:
            self.setBrush(QBrush(QColor(173, 216, 230)))  # Light blue
        elif plugin_info and plugin_info.plugin_type == "python":
            self.setBrush(QBrush(QColor(144, 238, 144)))  # Light green
        else:
            self.setBrush(QBrush(QColor(255, 182, 193)))  # Light pink

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
            type_text = (
                "SM"
                if is_state_machine
                else ("Py" if plugin_info.plugin_type == "python" else "C++")
            )
            self.type_label = QGraphicsTextItem(type_text, self)
            self.type_label.setDefaultTextColor(Qt.darkGray)
            type_font = QFont()
            type_font.setPointSize(8)
            self.type_label.setFont(type_font)
            type_rect = self.type_label.boundingRect()
            self.type_label.setPos(-type_rect.width() / 2, 10)

        # Add connection port (small circle on the right edge for drag-to-connect)
        self.connection_port = ConnectionPort(self)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            # Update all connections when the state moves
            for connection in self.connections:
                connection.update_position()
        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine"):
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine"):
        if connection in self.connections:
            self.connections.remove(connection)

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


class FinalOutcomeNode(QGraphicsRectItem):
    """Graphical representation of a final outcome."""

    def __init__(self, name: str, x: float, y: float):
        super().__init__(-60, -30, 120, 60)
        self.name = name
        self.connections: List["ConnectionLine"] = []

        # Set position
        self.setPos(x, y)

        # Set drag/move flags
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        # Set colors
        self.setBrush(QBrush(QColor(255, 0, 0)))  # Red
        self.setPen(QPen(QColor(0, 0, 0), 3))

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

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            # Update all connections when the state moves
            for connection in self.connections:
                connection.update_position()
        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine"):
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine"):
        if connection in self.connections:
            self.connections.remove(connection)

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the rectangle edge closest to target."""
        center = self.scenePos()
        angle = math.atan2(target_pos.y() - center.y(), target_pos.x() - center.x())
        # Rectangle dimensions
        w = 60
        h = 30
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


class ConnectionLine(QGraphicsPathItem):
    """Graphical representation of a transition between states with curved line."""

    def __init__(self, from_node, to_node, outcome: str):
        super().__init__()
        self.from_node = from_node
        self.to_node = to_node
        self.outcome = outcome

        # Set pen with nice styling
        pen = QPen(QColor(60, 60, 180), 3, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        self.setPen(pen)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        # Highlight pen for selection
        self.normal_pen = pen
        self.selected_pen = QPen(QColor(255, 100, 0), 4, Qt.SolidLine)
        self.selected_pen.setCapStyle(Qt.RoundCap)

        # Add arrow head as a separate polygon item
        self.arrow_head = QGraphicsPolygonItem()
        self.arrow_head.setBrush(QBrush(QColor(60, 60, 180)))
        self.arrow_head.setPen(QPen(QColor(60, 60, 180)))

        # Add label background
        self.label_bg = QGraphicsRectItem()
        self.label_bg.setBrush(QBrush(QColor(255, 255, 255, 230)))
        self.label_bg.setPen(QPen(QColor(60, 60, 180), 1))

        # Add label
        self.label = QGraphicsTextItem(outcome)
        self.label.setDefaultTextColor(QColor(0, 0, 100))
        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        self.label.setFont(font)

        # Update position
        self.update_position()

        # Add to nodes
        from_node.add_connection(self)
        to_node.add_connection(self)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemSelectedChange:
            # Change pen when selected
            if value:
                self.setPen(self.selected_pen)
                self.arrow_head.setBrush(QBrush(QColor(255, 100, 0)))
                self.arrow_head.setPen(QPen(QColor(255, 100, 0)))
            else:
                self.setPen(self.normal_pen)
                self.arrow_head.setBrush(QBrush(QColor(60, 60, 180)))
                self.arrow_head.setPen(QPen(QColor(60, 60, 180)))
        return super().itemChange(change, value)

    def update_position(self):
        """Update the connection line with a smooth curved path."""
        # Get connection points on the edges of nodes
        from_center = self.from_node.get_connection_point()
        to_center = self.to_node.get_connection_point()

        # Calculate edge points
        from_pos = self.from_node.get_edge_point(to_center)
        to_pos = self.to_node.get_edge_point(from_center)

        # Create curved path
        path = QPainterPath()
        path.moveTo(from_pos)

        # Calculate control points for bezier curve
        dx = to_pos.x() - from_pos.x()
        dy = to_pos.y() - from_pos.y()

        # Control point offset (creates the curve)
        offset = math.sqrt(dx * dx + dy * dy) * 0.3

        # Calculate angle for perpendicular offset
        angle = math.atan2(dy, dx)
        perp_angle = angle + math.pi / 2

        # Create control points slightly offset perpendicular to the line
        ctrl1 = QPointF(
            from_pos.x() + dx * 0.33 + math.cos(perp_angle) * offset * 0.3,
            from_pos.y() + dy * 0.33 + math.sin(perp_angle) * offset * 0.3,
        )
        ctrl2 = QPointF(
            from_pos.x() + dx * 0.67 - math.cos(perp_angle) * offset * 0.3,
            from_pos.y() + dy * 0.67 - math.sin(perp_angle) * offset * 0.3,
        )

        # Draw cubic bezier curve
        path.cubicTo(ctrl1, ctrl2, to_pos)

        self.setPath(path)

        # Calculate arrow head at the end point
        # Get the tangent at the end of the curve
        t = 0.95  # Sample point near the end
        tangent_point = path.pointAtPercent(t)
        angle = math.atan2(to_pos.y() - tangent_point.y(), to_pos.x() - tangent_point.x())

        arrow_size = 12
        arrow_p1 = to_pos - QPointF(
            math.cos(angle - math.pi / 6) * arrow_size,
            math.sin(angle - math.pi / 6) * arrow_size,
        )
        arrow_p2 = to_pos - QPointF(
            math.cos(angle + math.pi / 6) * arrow_size,
            math.sin(angle + math.pi / 6) * arrow_size,
        )

        arrow_polygon = QPolygonF([to_pos, arrow_p1, arrow_p2])
        self.arrow_head.setPolygon(arrow_polygon)

        # Update label position (middle of path)
        mid_point = path.pointAtPercent(0.5)
        label_rect = self.label.boundingRect()

        # Position label with background
        padding = 4
        self.label_bg.setRect(
            mid_point.x() - label_rect.width() / 2 - padding,
            mid_point.y() - label_rect.height() / 2 - padding,
            label_rect.width() + padding * 2,
            label_rect.height() + padding * 2,
        )
        self.label.setPos(
            mid_point.x() - label_rect.width() / 2,
            mid_point.y() - label_rect.height() / 2,
        )


class StatePropertiesDialog(QDialog):
    """Dialog for setting state properties."""

    def __init__(
        self,
        state_name: str = "",
        plugin_info: PluginInfo = None,
        available_plugins: List[PluginInfo] = None,
        available_xml: List[str] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("State Properties")
        self.resize(400, 300)

        layout = QFormLayout(self)

        # State name
        self.name_edit = QLineEdit(state_name)
        layout.addRow("State Name:", self.name_edit)

        # State type
        self.type_combo = QComboBox()
        self.type_combo.addItem("Python State")
        self.type_combo.addItem("C++ State")
        self.type_combo.addItem("State Machine (XML)")
        self.type_combo.currentIndexChanged.connect(self.on_type_changed)
        layout.addRow("State Type:", self.type_combo)

        # Plugin selection
        self.plugin_combo = QComboBox()
        self.available_plugins = available_plugins or []
        self.available_xml = available_xml or []

        if plugin_info:
            if plugin_info.plugin_type == "python":
                self.type_combo.setCurrentIndex(0)
            else:
                self.type_combo.setCurrentIndex(1)

        self.update_plugin_list()
        layout.addRow("Plugin/State Machine:", self.plugin_combo)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def on_type_changed(self):
        self.update_plugin_list()

    def update_plugin_list(self):
        self.plugin_combo.clear()

        current_type = self.type_combo.currentIndex()
        if current_type == 0:  # Python
            for plugin in self.available_plugins:
                if plugin.plugin_type == "python":
                    self.plugin_combo.addItem(
                        f"{plugin.module}.{plugin.class_name}", plugin
                    )
        elif current_type == 1:  # C++
            for plugin in self.available_plugins:
                if plugin.plugin_type == "cpp":
                    self.plugin_combo.addItem(plugin.class_name, plugin)
        else:  # State Machine
            for xml_plugin in self.available_xml:
                import os

                self.plugin_combo.addItem(
                    os.path.basename(xml_plugin.file_path), xml_plugin
                )

    def get_state_data(self) -> Tuple[str, PluginInfo, bool, Optional[str]]:
        name = self.name_edit.text()
        is_sm = self.type_combo.currentIndex() == 2

        if is_sm:
            xml_plugin = self.plugin_combo.currentData()
            return name, xml_plugin, True, xml_plugin.file_path if xml_plugin else None
        else:
            plugin = self.plugin_combo.currentData()
            return name, plugin, False, None


class TransitionDialog(QDialog):
    """Dialog for creating transitions between states."""

    def __init__(self, from_state: StateNode, available_targets: List, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Transition")
        self.resize(400, 200)

        layout = QFormLayout(self)

        # From state
        from_label = QLabel(f"<b>{from_state.name}</b>")
        layout.addRow("From State:", from_label)

        # Outcome
        self.outcome_combo = QComboBox()
        if from_state.plugin_info:
            for outcome in from_state.plugin_info.outcomes:
                self.outcome_combo.addItem(outcome)
        layout.addRow("Outcome:", self.outcome_combo)

        # To state
        self.target_combo = QComboBox()
        for target in available_targets:
            if isinstance(target, StateNode):
                self.target_combo.addItem(target.name, target)
            elif isinstance(target, FinalOutcomeNode):
                self.target_combo.addItem(f"[Final] {target.name}", target)
        layout.addRow("To State:", self.target_combo)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_transition_data(self) -> Tuple[str, object]:
        outcome = self.outcome_combo.currentText()
        target = self.target_combo.currentData()
        return outcome, target


class StateMachineCanvas(QGraphicsView):
    """Canvas for drawing the state machine with drag-to-connect support."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setSceneRect(-2000, -2000, 4000, 4000)

        # Set background
        self.setBackgroundBrush(QBrush(QColor(250, 250, 250)))

        # Drag-to-connect state
        self.drag_start_node = None
        self.temp_line = None
        self.editor_ref = None  # Will be set by YasminEditor
        self.is_dragging_connection = False

        # Enable dragging by default
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setMouseTracking(True)

        # Enable context menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_context_menu)

    def show_context_menu(self, position):
        """Show context menu with options to add states, transitions, etc."""
        if not self.editor_ref:
            return

        from PyQt5.QtWidgets import QMenu

        menu = QMenu(self)

        # Add State action
        add_state_action = menu.addAction("Add State")
        add_state_action.triggered.connect(self.editor_ref.add_state)

        # Add Transition action (only if a state is selected)
        selected_items = self.scene.selectedItems()
        has_selected_state = any(isinstance(item, StateNode) for item in selected_items)

        add_transition_action = menu.addAction("Add Transition")
        add_transition_action.setEnabled(has_selected_state)
        add_transition_action.triggered.connect(self.editor_ref.add_transition)

        # Add Final Outcome action
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
                if isinstance(scene_item, (StateNode, FinalOutcomeNode)):
                    scene_item.setOpacity(1.0)

            # Highlight if hovering over a valid target
            target = None
            if isinstance(item, (StateNode, FinalOutcomeNode)):
                target = item
            elif hasattr(item, "parentItem"):
                parent = item.parentItem()
                if isinstance(parent, (StateNode, FinalOutcomeNode)):
                    target = parent

            if target and target != self.drag_start_node:
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
                if isinstance(item, (StateNode, FinalOutcomeNode)):
                    target_node = item
                    break
                elif hasattr(item, "parentItem"):
                    parent = item.parentItem()
                    if isinstance(parent, (StateNode, FinalOutcomeNode)):
                        target_node = parent
                        break

            # Remove temporary line
            if self.temp_line:
                self.scene.removeItem(self.temp_line)
                self.temp_line = None

            # Reset opacity
            for scene_item in self.scene.items():
                if isinstance(scene_item, (StateNode, FinalOutcomeNode)):
                    scene_item.setOpacity(1.0)

            # Create connection if valid target
            # Capture references before resetting drag state
            source_node = self.drag_start_node

            # Reset drag state first
            self.drag_start_node = None
            self.is_dragging_connection = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

            # Now create connection with captured references
            if target_node and source_node and target_node != source_node:
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


class YasminEditor(QMainWindow):
    """Main editor window for YASMIN state machines."""

    def __init__(self, manager: PluginManager):
        super().__init__()
        self.setWindowTitle("YASMIN Editor")

        # Start maximized
        self.showMaximized()

        # Plugin manager
        self.plugin_manager = manager
        self.state_nodes: Dict[str, StateNode] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self.next_state_position = QPointF(0, 0)

        # Create UI
        self.create_ui()

        # Load plugins in background
        self.statusBar().showMessage("Loading plugins...")
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    def closeEvent(self, event):
        """Handle window close event to ensure proper cleanup."""
        # Clear all references
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()

        # Accept the close event
        event.accept()

        # Quit the application and exit the process
        QApplication.quit()

        # Force process termination
        import os

        os._exit(0)

    def create_ui(self):
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create splitter
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel - Available states
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Toolbar
        toolbar = QToolBar()
        self.addToolBar(toolbar)

        new_action = QAction("New", self)
        new_action.triggered.connect(self.new_state_machine)
        toolbar.addAction(new_action)

        open_action = QAction("Open", self)
        open_action.triggered.connect(self.open_state_machine)
        toolbar.addAction(open_action)

        save_action = QAction("Save", self)
        save_action.triggered.connect(self.save_state_machine)
        toolbar.addAction(save_action)

        toolbar.addSeparator()

        add_state_action = QAction("Add State", self)
        add_state_action.triggered.connect(self.add_state)
        toolbar.addAction(add_state_action)

        add_transition_action = QAction("Add Transition", self)
        add_transition_action.triggered.connect(self.add_transition)
        toolbar.addAction(add_transition_action)

        add_final_action = QAction("Add Final Outcome", self)
        add_final_action.triggered.connect(self.add_final_outcome)
        toolbar.addAction(add_final_action)

        toolbar.addSeparator()

        delete_action = QAction("Delete Selected", self)
        delete_action.triggered.connect(self.delete_selected)
        toolbar.addAction(delete_action)

        # Python states list
        left_layout.addWidget(QLabel("<b>Python States:</b>"))
        self.python_filter = QLineEdit()
        self.python_filter.setPlaceholderText("Filter Python states...")
        self.python_filter.textChanged.connect(self.filter_python_list)
        left_layout.addWidget(self.python_filter)
        self.python_list = QListWidget()
        self.python_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.python_list)

        # C++ states list
        left_layout.addWidget(QLabel("<b>C++ States:</b>"))
        self.cpp_filter = QLineEdit()
        self.cpp_filter.setPlaceholderText("Filter C++ states...")
        self.cpp_filter.textChanged.connect(self.filter_cpp_list)
        left_layout.addWidget(self.cpp_filter)
        self.cpp_list = QListWidget()
        self.cpp_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.cpp_list)

        # XML state machines list
        left_layout.addWidget(QLabel("<b>XML State Machines:</b>"))
        self.xml_filter = QLineEdit()
        self.xml_filter.setPlaceholderText("Filter XML state machines...")
        self.xml_filter.textChanged.connect(self.filter_xml_list)
        left_layout.addWidget(self.xml_filter)
        self.xml_list = QListWidget()
        self.xml_list.itemDoubleClicked.connect(self.on_xml_double_clicked)
        left_layout.addWidget(self.xml_list)

        # Right panel - Canvas
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # Canvas header with instructions
        canvas_header = QLabel(
            "<b>State Machine Canvas:</b> "
            "<i>(Drag from blue port to create transitions, scroll to zoom, right-click for options)</i>"
        )
        right_layout.addWidget(canvas_header)
        self.canvas = StateMachineCanvas()
        self.canvas.editor_ref = self  # Set reference for drag-to-connect
        right_layout.addWidget(self.canvas)

        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        # Set initial sizes: left panel 300px, right panel takes remaining space
        splitter.setSizes([300, 1000])
        splitter.setStretchFactor(0, 0)  # Left panel doesn't stretch
        splitter.setStretchFactor(1, 1)  # Right panel stretches

        # Status bar
        self.statusBar()

    def populate_plugin_lists(self):
        # Populate Python list
        for plugin in self.plugin_manager.python_plugins:
            item = QListWidgetItem(f"{plugin.module}.{plugin.class_name}")
            item.setData(Qt.UserRole, plugin)
            self.python_list.addItem(item)

        # Populate C++ list
        for plugin in self.plugin_manager.cpp_plugins:
            item = QListWidgetItem(plugin.class_name)
            item.setData(Qt.UserRole, plugin)
            self.cpp_list.addItem(item)

        # Populate XML list
        for xml_plugin in self.plugin_manager.xml_files:
            item = QListWidgetItem(os.path.basename(xml_plugin.file_path))
            item.setData(Qt.UserRole, xml_plugin)
            self.xml_list.addItem(item)

    def filter_python_list(self, text):
        """Filter Python states list based on search text."""
        for i in range(self.python_list.count()):
            item = self.python_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_cpp_list(self, text):
        """Filter C++ states list based on search text."""
        for i in range(self.cpp_list.count()):
            item = self.cpp_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_xml_list(self, text):
        """Filter XML state machines list based on search text."""
        for i in range(self.xml_list.count()):
            item = self.xml_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def on_plugin_double_clicked(self, item: QListWidgetItem):
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok and state_name:
            self.create_state_node(state_name, plugin_info, False)

    def on_xml_double_clicked(self, item: QListWidgetItem):
        xml_plugin = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name:"
        )
        if ok and state_name:
            # For XML state machines, pass the plugin_info
            self.create_state_node(state_name, xml_plugin, True, xml_plugin.file_path)

    def create_state_node(
        self,
        name: str,
        plugin_info: PluginInfo,
        is_state_machine: bool = False,
        xml_file: str = None,
    ):
        if name in self.state_nodes:
            QMessageBox.warning(self, "Error", f"State '{name}' already exists!")
            return

        # Create node
        node = StateNode(
            name,
            plugin_info,
            self.next_state_position.x(),
            self.next_state_position.y(),
            is_state_machine,
        )

        # Store XML file reference if applicable
        if xml_file:
            node.xml_file = xml_file

        self.canvas.scene.addItem(node)
        self.state_nodes[name] = node

        # Update position for next state
        self.next_state_position += QPointF(150, 100)

        self.statusBar().showMessage(f"Added state: {name}", 2000)

    def add_state(self):
        all_plugins = self.plugin_manager.python_plugins + self.plugin_manager.cpp_plugins
        dialog = StatePropertiesDialog(
            available_plugins=all_plugins,
            available_xml=self.plugin_manager.xml_files,
            parent=self,
        )

        if dialog.exec_():
            name, plugin, is_sm, xml_file = dialog.get_state_data()
            if name:
                self.create_state_node(name, plugin, is_sm, xml_file)

    def create_connection_from_drag(self, from_node: StateNode, to_node):
        """Create a connection when user drags from one node to another."""
        if not from_node.plugin_info:
            QMessageBox.warning(
                self,
                "Error",
                "Cannot create transitions from state machines without outcomes!",
            )
            return

        # If the from_node has only one outcome, use it directly
        if len(from_node.plugin_info.outcomes) == 1:
            outcome = list(from_node.plugin_info.outcomes)[0]
            self.create_connection(from_node, to_node, outcome)
        else:
            # Ask user to select outcome
            outcome, ok = QInputDialog.getItem(
                self,
                "Select Outcome",
                f"Select outcome for transition from '{from_node.name}':",
                list(from_node.plugin_info.outcomes),
                0,
                False,
            )
            if ok:
                self.create_connection(from_node, to_node, outcome)

    def create_connection(self, from_node, to_node, outcome: str):
        """Create and add a connection to the scene."""
        connection = ConnectionLine(from_node, to_node, outcome)
        self.canvas.scene.addItem(connection)
        self.canvas.scene.addItem(connection.arrow_head)
        self.canvas.scene.addItem(connection.label_bg)
        self.canvas.scene.addItem(connection.label)
        self.connections.append(connection)
        self.statusBar().showMessage(
            f"Added transition: {from_node.name} --[{outcome}]--> {to_node.name}",
            2000,
        )

    def add_transition(self):
        # Get selected state
        selected_items = self.canvas.scene.selectedItems()
        from_state = None

        for item in selected_items:
            if isinstance(item, StateNode):
                from_state = item
                break

        if not from_state:
            QMessageBox.warning(self, "Error", "Please select a source state first!")
            return

        if not from_state.plugin_info:
            QMessageBox.warning(
                self,
                "Error",
                "Cannot create transitions from state machines without outcomes!",
            )
            return

        # Get available targets
        available_targets = list(self.state_nodes.values()) + list(
            self.final_outcomes.values()
        )
        available_targets = [t for t in available_targets if t != from_state]

        if not available_targets:
            QMessageBox.warning(self, "Error", "No target states available!")
            return

        dialog = TransitionDialog(from_state, available_targets, self)

        if dialog.exec_():
            outcome, target = dialog.get_transition_data()
            self.create_connection(from_state, target, outcome)

    def add_final_outcome(self):
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        if ok and outcome_name:
            if outcome_name in self.final_outcomes:
                QMessageBox.warning(
                    self, "Error", f"Final outcome '{outcome_name}' already exists!"
                )
                return

            # Create final outcome node (positioned on the right side)
            x = self.next_state_position.x() + 300
            y = len(self.final_outcomes) * 150
            node = FinalOutcomeNode(outcome_name, x, y)
            self.canvas.scene.addItem(node)
            self.final_outcomes[outcome_name] = node
            self.statusBar().showMessage(f"Added final outcome: {outcome_name}", 2000)

    def delete_selected(self):
        selected_items = self.canvas.scene.selectedItems()

        for item in selected_items:
            if isinstance(item, StateNode):
                # Remove all connections
                for connection in item.connections[:]:
                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                # Remove node
                self.canvas.scene.removeItem(item)
                del self.state_nodes[item.name]
                self.statusBar().showMessage(f"Deleted state: {item.name}", 2000)

            elif isinstance(item, FinalOutcomeNode):
                # Remove all connections
                for connection in item.connections[:]:
                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                # Remove node
                self.canvas.scene.removeItem(item)
                del self.final_outcomes[item.name]
                self.statusBar().showMessage(f"Deleted final outcome: {item.name}", 2000)

            elif isinstance(item, ConnectionLine):
                item.from_node.remove_connection(item)
                item.to_node.remove_connection(item)
                self.canvas.scene.removeItem(item)
                self.canvas.scene.removeItem(item.arrow_head)
                self.canvas.scene.removeItem(item.label_bg)
                self.canvas.scene.removeItem(item.label)
                if item in self.connections:
                    self.connections.remove(item)
                self.statusBar().showMessage("Deleted transition", 2000)

    def new_state_machine(self):
        reply = QMessageBox.question(
            self,
            "New State Machine",
            "Are you sure you want to create a new state machine? All unsaved changes will be lost.",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self.canvas.scene.clear()
            self.state_nodes.clear()
            self.final_outcomes.clear()
            self.connections.clear()
            self.next_state_position = QPointF(0, 0)
            self.statusBar().showMessage("New state machine created", 2000)

    def open_state_machine(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            try:
                self.load_from_xml(file_path)
                self.statusBar().showMessage(f"Opened: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to open file: {str(e)}")

    def save_state_machine(self):
        if not self.final_outcomes:
            QMessageBox.warning(self, "Error", "Please add at least one final outcome!")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            try:
                self.save_to_xml(file_path)
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def save_to_xml(self, file_path: str):
        import xml.etree.ElementTree as ET
        from xml.dom import minidom

        # Get state machine name
        sm_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name (optional):"
        )

        # Create root element
        root = ET.Element("StateMachine")
        root.set("outcomes", " ".join(self.final_outcomes.keys()))
        if ok and sm_name:
            root.set("name", sm_name)

        # Add states
        for state_name, state_node in self.state_nodes.items():
            if state_node.is_state_machine:
                # Nested state machine
                sm_elem = ET.SubElement(root, "StateMachine")
                sm_elem.set("name", state_name)

                # Get outcomes from XML file if available
                if hasattr(state_node, "xml_file") and state_node.xml_file:
                    # Parse XML to get outcomes
                    import xml.etree.ElementTree as ET_parse

                    tree = ET_parse.parse(state_node.xml_file)
                    xml_root = tree.getroot()
                    outcomes = xml_root.get("outcomes", "")
                    if outcomes:
                        sm_elem.set("outcomes", outcomes)
            else:
                state_elem = ET.SubElement(root, "State")
                state_elem.set("name", state_name)

                if state_node.plugin_info:
                    if state_node.plugin_info.plugin_type == "python":
                        state_elem.set("type", "py")
                        state_elem.set("module", state_node.plugin_info.module)
                        state_elem.set("class", state_node.plugin_info.class_name)
                    else:
                        state_elem.set("type", "cpp")
                        state_elem.set("class", state_node.plugin_info.class_name)

            # Add transitions
            for connection in state_node.connections:
                if connection.from_node == state_node:
                    transition = ET.SubElement(
                        state_elem if not state_node.is_state_machine else sm_elem,
                        "Transition",
                    )
                    transition.set("from", connection.outcome)
                    transition.set("to", connection.to_node.name)

        # Pretty print XML
        xml_str = ET.tostring(root, encoding="unicode")
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")

        # Remove extra blank lines
        pretty_xml = "\n".join([line for line in pretty_xml.split("\n") if line.strip()])

        # Write to file
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(pretty_xml)

    def load_from_xml(self, file_path: str):
        import xml.etree.ElementTree as ET

        # Clear current state machine
        self.new_state_machine()

        # Parse XML
        tree = ET.parse(file_path)
        root = tree.getroot()

        # Get final outcomes
        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()
            for i, outcome in enumerate(outcomes):
                x = 600
                y = i * 150
                node = FinalOutcomeNode(outcome, x, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome] = node

        # Load states
        y_offset = 0
        for elem in root:
            if elem.tag == "State":
                state_name = elem.get("name")
                state_type = elem.get("type")

                # Find plugin
                plugin_info = None
                if state_type == "py":
                    module = elem.get("module")
                    class_name = elem.get("class")
                    for plugin in self.plugin_manager.python_plugins:
                        if plugin.module == module and plugin.class_name == class_name:
                            plugin_info = plugin
                            break
                elif state_type == "cpp":
                    class_name = elem.get("class")
                    for plugin in self.plugin_manager.cpp_plugins:
                        if plugin.class_name == class_name:
                            plugin_info = plugin
                            break

                if plugin_info:
                    node = StateNode(state_name, plugin_info, 0, y_offset, False)
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                    y_offset += 150

            elif elem.tag == "StateMachine":
                state_name = elem.get("name")
                # Create state machine node
                # Note: Nested state machines from XML don't have full plugin_info
                # Their outcomes are defined in the XML transitions
                node = StateNode(state_name, None, 0, y_offset, True)
                self.canvas.scene.addItem(node)
                self.state_nodes[state_name] = node
                y_offset += 150

        # Load transitions
        for elem in root:
            if elem.tag in ["State", "StateMachine"]:
                state_name = elem.get("name")
                from_node = self.state_nodes.get(state_name)

                if from_node:
                    for transition in elem.findall("Transition"):
                        outcome = transition.get("from")
                        to_name = transition.get("to")

                        # Find target node
                        to_node = self.state_nodes.get(to_name)
                        if not to_node:
                            to_node = self.final_outcomes.get(to_name)

                        if to_node:
                            connection = ConnectionLine(from_node, to_node, outcome)
                            self.canvas.scene.addItem(connection)
                            self.canvas.scene.addItem(connection.arrow_head)
                            self.canvas.scene.addItem(connection.label_bg)
                            self.canvas.scene.addItem(connection.label)
                            self.connections.append(connection)


def main():
    import atexit

    # Load plugins before creating QApplication
    manager = PluginManager()
    manager.load_all_plugins()

    app = QApplication(sys.argv)

    # Set application to quit when last window is closed
    app.setQuitOnLastWindowClosed(True)

    # Register cleanup on exit
    def cleanup():
        if app:
            app.quit()

    atexit.register(cleanup)

    editor = YasminEditor(manager)
    editor.show()

    # Execute the application and return exit code
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
