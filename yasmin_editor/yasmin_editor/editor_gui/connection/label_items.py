# Copyright (C) 2026 Maik Knof
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

"""Clickable label items used by connection lines."""

from typing import TYPE_CHECKING, Any

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsTextItem

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine


class ConnectionLabelRectItem(QGraphicsRectItem):
    """Clickable label background that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine") -> None:
        super().__init__()
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)


class ConnectionLabelTextItem(QGraphicsTextItem):
    """Clickable label text that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine", text: str) -> None:
        super().__init__(text)
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)
