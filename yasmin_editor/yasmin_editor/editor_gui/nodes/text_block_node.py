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
"""Inline-editable free-form text block for the editor canvas."""

from __future__ import annotations

from typing import Any, Optional

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QFont, QKeySequence, QPen, QTextCursor, QTextDocument
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsRectItem, QGraphicsTextItem, QMenu

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.text_block import TextBlock


class TextBlockEditorItem(QGraphicsTextItem):
    """Text item that provides inline editing helpers for text blocks."""

    def __init__(self, node: "TextBlockNode") -> None:
        super().__init__(node)
        self.node = node
        self.setTabChangesFocus(False)
        self.setTextInteractionFlags(Qt.NoTextInteraction)
        self.setAcceptedMouseButtons(Qt.NoButton)

    def focusOutEvent(self, event: Any) -> None:
        super().focusOutEvent(event)
        self.node.finish_editing()

    def keyPressEvent(self, event: Any) -> None:
        if event.key() == Qt.Key_Escape:
            self.node.cancel_editing()
            event.accept()
            return

        if event.matches(QKeySequence.Bold):
            self._wrap_selection("**", "**")
            event.accept()
            return

        if event.matches(QKeySequence.Italic):
            self._wrap_selection("_", "_")
            event.accept()
            return

        if event.key() == Qt.Key_Return and bool(event.modifiers() & Qt.ControlModifier):
            self.node.finish_editing()
            event.accept()
            return

        super().keyPressEvent(event)
        self.node.update_geometry_from_document()

    def _wrap_selection(self, prefix: str, suffix: str) -> None:
        cursor = self.textCursor()
        selected_text = cursor.selectedText().replace("\u2029", "\n")

        if selected_text:
            cursor.insertText(f"{prefix}{selected_text}{suffix}")
        else:
            cursor.insertText(f"{prefix}{suffix}")
            for _ in range(len(suffix)):
                cursor.movePosition(QTextCursor.Left)
            self.setTextCursor(cursor)

        self.node.update_geometry_from_document()


class TextBlockNode(QGraphicsRectItem, BaseNodeMixin):
    """Graphical free-form text block with inline editing and Markdown preview."""

    _PADDING_X = 12.0
    _PADDING_Y = 10.0
    _MIN_WIDTH = 0.0
    _MIN_HEIGHT = 0.0
    _BASE_FONT_POINT_SIZE = 10

    def __init__(
        self, x: float, y: float, content: str = "", model: Optional[TextBlock] = None
    ) -> None:
        super().__init__()
        self.model: TextBlock = model or TextBlock(x=x, y=y, content=content)
        self.placement_label = "Text"
        self._is_editing = False
        self._cached_content_before_edit = self.model.content

        self._initialize_base_node_graphics(x, y)
        self.setZValue(-1.0)
        self.setBrush(QBrush(PALETTE.ui_panel_alt_bg))
        self.setPen(self._default_pen())

        self.text_item = TextBlockEditorItem(self)
        self.text_item.setDefaultTextColor(PALETTE.text_primary)
        self._base_font = QFont()
        self._base_font.setPointSize(self._BASE_FONT_POINT_SIZE)
        self.text_item.setFont(self._base_font)

        self.update_content_view()
        self.update_tooltip()

    @property
    def content(self) -> str:
        """Return the raw persisted text content."""
        return self.model.content

    @content.setter
    def content(self, value: str) -> None:
        """Update the raw persisted text content and refresh the rendered view."""
        self.model.content = value
        self.update_content_view()
        self.update_tooltip()

    def _default_pen(self) -> QPen:
        pen = QPen(PALETTE.ui_border, 1, Qt.DashLine)
        return pen

    def update_tooltip(self) -> None:
        """Update the tooltip with editing hints and a short content preview."""
        preview = self.model.content.strip() or "Empty text block"
        self.setToolTip(
            f"{preview}\n\nDouble-click to edit inline. Ctrl+B / Ctrl+I insert Markdown markers. Ctrl+Enter finishes editing."
        )

    def _create_document(self, content: str, markdown_enabled: bool) -> QTextDocument:
        """Create a fresh text document for the requested rendering mode."""
        document = QTextDocument(self.text_item)
        document.setDocumentMargin(0.0)
        document.setDefaultFont(self._base_font)

        if markdown_enabled and hasattr(document, "setMarkdown"):
            document.setMarkdown(content)
        else:
            document.setPlainText(content)

        return document

    def _apply_rendered_content(self) -> None:
        """Render the stored content in preview mode using Markdown when available."""
        self.text_item.setDocument(
            self._create_document(self.model.content, markdown_enabled=True)
        )
        self.text_item.setTextWidth(-1)

    def _apply_edit_content(self) -> None:
        """Render the stored content in edit mode as raw plain text."""
        self.text_item.setDocument(
            self._create_document(self.model.content, markdown_enabled=False)
        )
        self.text_item.setTextWidth(-1)

    def update_content_view(self) -> None:
        """Refresh the displayed text based on the current edit mode."""
        if self._is_editing:
            self._apply_edit_content()
            self.text_item.setTextInteractionFlags(Qt.TextEditorInteraction)
            self.text_item.setAcceptedMouseButtons(Qt.AllButtons)
        else:
            self.text_item.setTextInteractionFlags(Qt.NoTextInteraction)
            self.text_item.setAcceptedMouseButtons(Qt.NoButton)
            self._apply_rendered_content()
        self.update_geometry_from_document()

    def update_geometry_from_document(self) -> None:
        """Resize the block to match the current document size."""
        text_rect = self.text_item.boundingRect()
        width = max(self._MIN_WIDTH, text_rect.width() + 2.0 * self._PADDING_X)
        height = max(self._MIN_HEIGHT, text_rect.height() + 2.0 * self._PADDING_Y)
        self.prepareGeometryChange()
        self.setRect(QRectF(0.0, 0.0, width, height))
        self.text_item.setPos(self._PADDING_X, self._PADDING_Y)

    def enter_edit_mode(self) -> None:
        """Switch the block into inline edit mode."""
        if self._is_editing:
            return

        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if (
                hasattr(canvas, "editor_ref")
                and canvas.editor_ref
                and canvas.editor_ref.is_read_only_mode()
            ):
                return

        self._cached_content_before_edit = self.model.content
        self._is_editing = True
        self.setSelected(True)
        self.update_content_view()
        self.text_item.setFocus(Qt.MouseFocusReason)
        cursor = self.text_item.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.text_item.setTextCursor(cursor)

    def finish_editing(self) -> None:
        """Persist inline edits and switch back to preview mode."""
        if not self._is_editing:
            return
        previous_content = self.model.content
        self.model.content = self.text_item.toPlainText()
        self._is_editing = False
        self.update_content_view()
        self.update_tooltip()

        if (
            previous_content != self.model.content
            and self.scene()
            and self.scene().views()
        ):
            canvas = self.scene().views()[0]
            editor_ref = getattr(canvas, "editor_ref", None)
            if editor_ref is not None:
                editor_ref.record_history_checkpoint()

    def cancel_editing(self) -> None:
        """Discard the current inline edits and restore the last saved content."""
        if not self._is_editing:
            return
        self.model.content = self._cached_content_before_edit
        self._is_editing = False
        self.update_content_view()
        self.update_tooltip()

    def set_read_only(self, readonly: bool) -> None:
        """Apply the current read-only mode to the text block."""
        if readonly and self._is_editing:
            self.finish_editing()
        self.setFlag(QGraphicsItem.ItemIsMovable, not readonly)
        if readonly:
            self.text_item.setTextInteractionFlags(Qt.NoTextInteraction)
            self.text_item.setAcceptedMouseButtons(Qt.NoButton)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Enter inline editing when the block is double-clicked."""
        self.enter_edit_mode()
        event.accept()

    def contextMenuEvent(self, event: Any) -> None:
        """Show a small context menu for inline editing and deletion."""
        editor_ref = None
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref"):
                editor_ref = canvas.editor_ref

        readonly = bool(editor_ref and editor_ref.is_read_only_mode())
        menu = QMenu()
        edit_action = menu.addAction("View Text" if readonly else "Edit Text")
        delete_action = None if readonly else menu.addAction("Delete")
        action = menu.exec_(event.screenPos())

        if action == edit_action:
            self.setSelected(True)
            if not readonly:
                self.enter_edit_mode()
            event.accept()
            return

        if action == delete_action and editor_ref is not None:
            self.setSelected(True)
            editor_ref.delete_selected()
            event.accept()
            return

        super().contextMenuEvent(event)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            value = self.constrain_position_to_parent(value, top_margin=10.0)

        elif change == QGraphicsItem.ItemSelectedChange:
            self.update_selection_pen(bool(value), self._default_pen())

        return super().itemChange(change, value)
