# Copyright (C) 2026 Maik Knof
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

from typing import TYPE_CHECKING, Any

from yasmin_editor.qt_compat import Qt, QtWidgets

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine


class ConnectionLabelRectItem(QtWidgets.QGraphicsRectItem):
    """Clickable label background that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine") -> None:
        super().__init__()
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)


class ConnectionLabelTextItem(QtWidgets.QGraphicsTextItem):
    """Clickable label text that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine", text: str) -> None:
        super().__init__(text)
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)
