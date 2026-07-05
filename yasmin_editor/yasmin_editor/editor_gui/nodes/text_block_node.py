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

from __future__ import annotations

from typing import Any, Optional

from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets, exec_menu
from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.text_block import TextBlock


class TextBlockEditorItem(QtWidgets.QGraphicsTextItem):
    """Text item that provides inline editing helpers for text blocks."""

    def __init__(self, node: "TextBlockNode") -> None:
        super().__init__(node)
        self.node = node
        self.setTabChangesFocus(False)
        self.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)
        self.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def focusOutEvent(self, event: Any) -> None:
        super().focusOutEvent(event)
        self.node.finish_editing()

    def keyPressEvent(self, event: Any) -> None:
        if event.key() == Qt.Key.Key_Escape:
            self.node.cancel_editing()
            event.accept()
            return

        if event.matches(QtGui.QKeySequence.StandardKey.Bold):
            self._wrap_selection("**", "**")
            event.accept()
            return

        if event.matches(QtGui.QKeySequence.StandardKey.Italic):
            self._wrap_selection("_", "_")
            event.accept()
            return

        if event.key() == Qt.Key.Key_Return and bool(
            event.modifiers() & Qt.KeyboardModifier.ControlModifier
        ):
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
                cursor.movePosition(QtGui.QTextCursor.Left)
            self.setTextCursor(cursor)

        self.node.update_geometry_from_document()


class TextBlockNode(BaseNodeMixin, QtWidgets.QGraphicsRectItem):
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
        self.setBrush(QtGui.QBrush(PALETTE.ui_panel_alt_bg))
        self.setPen(self._default_pen())

        self.text_item = TextBlockEditorItem(self)
        self.text_item.setDefaultTextColor(PALETTE.text_primary)
        self._base_font = QtGui.QFont()
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

    def _default_pen(self) -> QtGui.QPen:
        pen = QtGui.QPen(PALETTE.ui_border, 1, Qt.PenStyle.DashLine)
        return pen

    def update_tooltip(self) -> None:
        """Update the tooltip with editing hints and a short content preview."""
        preview = self.model.content.strip() or "Empty text block"
        self.setToolTip(
            f"{preview}\n\nDouble-click to edit inline. Ctrl+B / Ctrl+I insert Markdown markers. Ctrl+Enter finishes editing."
        )

    def _create_document(
        self, content: str, markdown_enabled: bool
    ) -> QtGui.QTextDocument:
        """Create a fresh text document for the requested rendering mode."""
        document = QtGui.QTextDocument(self.text_item)
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
            self.text_item.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextEditorInteraction
            )
            self.text_item.setAcceptedMouseButtons(Qt.MouseButton.AllButtons)
        else:
            self.text_item.setTextInteractionFlags(
                Qt.TextInteractionFlag.NoTextInteraction
            )
            self.text_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self._apply_rendered_content()
        self.update_geometry_from_document()

    def update_geometry_from_document(self) -> None:
        """Resize the block to match the current document size."""
        text_rect = self.text_item.boundingRect()
        width = max(self._MIN_WIDTH, text_rect.width() + 2.0 * self._PADDING_X)
        height = max(self._MIN_HEIGHT, text_rect.height() + 2.0 * self._PADDING_Y)
        self.prepareGeometryChange()
        self.setRect(QtCore.QRectF(0.0, 0.0, width, height))
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
        self.text_item.setFocus(Qt.FocusReason.MouseFocusReason)
        cursor = self.text_item.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
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
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable, not readonly)
        if readonly:
            self.text_item.setTextInteractionFlags(
                Qt.TextInteractionFlag.NoTextInteraction
            )
            self.text_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Enter inline editing when the block is double-clicked."""
        self.enter_edit_mode()
        event.accept()

    def _on_context_menu(self, editor: Any, event: Any) -> bool:
        readonly = editor.is_read_only_mode()
        menu = QtWidgets.QMenu()
        edit_action = menu.addAction("View Text" if readonly else "Edit Text")
        delete_action = None if readonly else menu.addAction("Delete")
        action = exec_menu(menu, event.screenPos())

        if action == edit_action:
            if not readonly:
                self.enter_edit_mode()
            return True

        if action == delete_action:
            editor.delete_selected()
            return True
        return False

    def itemChange(
        self, change: QtWidgets.QGraphicsItem.GraphicsItemChange, value: Any
    ) -> Any:
        if (
            change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and isinstance(value, QtCore.QPointF)
        ):
            value = self.constrain_position_to_parent(value, top_margin=10.0)

        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedChange:
            self.update_selection_pen(bool(value), self._default_pen())

        return super().itemChange(change, value)
