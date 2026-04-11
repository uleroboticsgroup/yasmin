#!/usr/bin/env python3
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
"""Shelf split-view and selection transfer helpers."""

from __future__ import annotations

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtWidgets import QInputDialog, QMessageBox, QWidget

from yasmin_editor.editor_gui.clipboard_logic import (
    cross_container_paste_warning,
    replacement_clipboard_message,
)
from yasmin_editor.editor_gui.clipboard_model import (
    create_clipboard_container,
    get_container_kind,
    is_container_empty,
)
from yasmin_editor.editor_gui.layout_sync import sync_container_layout_from_views
from yasmin_editor.editor_gui.scene_renderer import (
    SceneRenderContext,
    create_connection_view,
    ensure_outcome_placements,
    render_container_scene,
)
from yasmin_editor.editor_gui.scene_selection import collect_scene_selection
from yasmin_editor.editor_gui.selection_bundle_collect import collect_selection_bundle
from yasmin_editor.editor_gui.selection_bundle_geometry import get_bundle_bounds
from yasmin_editor.editor_gui.selection_bundle_paste import paste_bundle_into_model
from yasmin_editor.editor_gui.selection_bundle_remove import remove_selection_from_model
from yasmin_editor.editor_gui.selection_placement import build_selection_preview
from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import (
    MIN_CLIPBOARD_DOCK_WIDTH,
    clamp_clipboard_dock_width,
)
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state_machine import StateMachine


class EditorClipboardMixin:
    """Mixin that owns the editor shelf workflow."""

    def _switch_clipboard_model(self, kind: str) -> None:
        self.clipboard_model = create_clipboard_container(kind)
        self.refresh_clipboard_canvas()

    def _ensure_clipboard_model_kind(self, required_kind: str) -> bool:
        current_kind = get_container_kind(self.clipboard_model)
        if current_kind == required_kind:
            return True
        if is_container_empty(self.clipboard_model):
            self._switch_clipboard_model(required_kind)
            return True
        reply = QMessageBox.question(
            self,
            "Replace Shelf Content",
            replacement_clipboard_message(required_kind),
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return False
        self._switch_clipboard_model(required_kind)
        return True

    def _confirm_cross_container_paste(self, bundle) -> bool:
        warning_text = cross_container_paste_warning(
            bundle.source_kind,
            get_container_kind(self.current_container_model),
            has_transitions=bool(bundle.transitions),
            has_outcome_rules=bool(bundle.outcome_rules),
        )
        if warning_text is None:
            return True
        reply = QMessageBox.question(
            self,
            "Place Across Container Types",
            warning_text,
            QMessageBox.Yes | QMessageBox.No,
        )
        return reply == QMessageBox.Yes

    def sync_clipboard_layout(self) -> None:
        sync_container_layout_from_views(
            self.clipboard_model,
            self.clipboard_state_nodes,
            self.clipboard_final_outcomes.values(),
            self.clipboard_text_blocks,
        )

    @staticmethod
    def _bundle_from_scene_selection(container_model, selection):
        return collect_selection_bundle(
            container_model,
            selection.state_names,
            selection.outcome_instance_ids,
            selection.text_models,
        )

    @staticmethod
    def _remove_scene_selection_from_model(container_model, selection) -> None:
        remove_selection_from_model(
            container_model,
            selection.state_names,
            selection.outcome_instance_ids,
            selection.text_models,
        )

    def _current_scene_selection(self):
        """Return the current main-canvas selection after syncing live positions."""

        self.sync_current_container_layout()
        return collect_scene_selection(self.canvas.scene.selectedItems())

    def _clipboard_scene_selection(self, fallback_to_all: bool = False):
        """Return the current shelf selection after syncing live positions."""

        self.sync_clipboard_layout()
        selected_items = self.clipboard_canvas.scene.selectedItems()
        if fallback_to_all and not selected_items:
            selected_items = (
                list(self.clipboard_state_nodes.values())
                + list(self.clipboard_final_outcomes.values())
                + list(self.clipboard_text_blocks)
            )
        return collect_scene_selection(selected_items)

    def _collect_main_selection_bundle(self):
        selection = self._current_scene_selection()
        return self._bundle_from_scene_selection(self.current_container_model, selection)

    def _collect_clipboard_selection_bundle(self, fallback_to_all: bool = False):
        selection = self._clipboard_scene_selection(fallback_to_all=fallback_to_all)
        return self._bundle_from_scene_selection(self.clipboard_model, selection)

    def _clear_clipboard_scene(self) -> None:
        self.clipboard_canvas.clear_pending_placement()
        self.clipboard_canvas.scene.clear()
        self.clipboard_state_nodes.clear()
        self.clipboard_final_outcomes.clear()
        self.clipboard_connections.clear()
        self.clipboard_text_blocks.clear()

    def _resolve_clipboard_target_view(
        self,
        target_name: str,
        target_instance_id: str = "",
    ):
        target = self.clipboard_state_nodes.get(target_name)
        if target is not None:
            return target
        if target_instance_id:
            instance_view = self.clipboard_final_outcomes.get(target_instance_id)
            if instance_view is not None:
                return instance_view
        for outcome_view in self.clipboard_final_outcomes.values():
            if outcome_view.name == target_name:
                return outcome_view
        return None

    def _create_clipboard_connection_view(self, from_node, to_node, outcome: str):
        return create_connection_view(
            self.clipboard_canvas.scene,
            self.clipboard_connections,
            from_node,
            to_node,
            outcome,
        )

    def _register_clipboard_state_node(self, node) -> None:
        self.clipboard_state_nodes[node.name] = node

    def _build_clipboard_scene_context(self) -> SceneRenderContext:
        return SceneRenderContext(
            scene=self.clipboard_canvas.scene,
            state_nodes=self.clipboard_state_nodes,
            final_outcomes=self.clipboard_final_outcomes,
            connections=self.clipboard_connections,
            text_blocks=self.clipboard_text_blocks,
            clear_scene=self._clear_clipboard_scene,
            register_state_node=self._register_clipboard_state_node,
            resolve_plugin_info=self.resolve_plugin_info_for_model,
            resolve_target_view=self._resolve_clipboard_target_view,
            resolve_primary_outcome_view=lambda outcome_name: self._resolve_clipboard_target_view(
                outcome_name,
                "",
            ),
            create_connection_view=self._create_clipboard_connection_view,
            ensure_outcome_placements=ensure_outcome_placements,
        )

    def _visible_clipboard_scene_rect(self) -> QRectF | None:
        """Return the bounds of visible shelf items only."""

        canvas = getattr(self, "clipboard_canvas", None)
        if canvas is None:
            return None
        visible_items = [item for item in canvas.scene.items() if item.isVisible()]
        if not visible_items:
            return None
        bounds = visible_items[0].sceneBoundingRect()
        for item in visible_items[1:]:
            bounds = bounds.united(item.sceneBoundingRect())
        return bounds

    def _remember_clipboard_panel_width(self, width: int | None = None) -> None:
        """Persist the current shelf width for hide/show toggles and restarts."""

        dock = getattr(self, "clipboard_dock", None)
        panel = getattr(self, "clipboard_panel", None)
        inferred_width = width
        if inferred_width is None:
            if dock is not None and dock.width() > 0:
                inferred_width = dock.width()
            elif panel is not None and panel.width() > 0:
                inferred_width = panel.width()
            else:
                inferred_width = getattr(
                    self,
                    "_clipboard_panel_width",
                    MIN_CLIPBOARD_DOCK_WIDTH,
                )
        normalized_width = clamp_clipboard_dock_width(inferred_width)
        self._clipboard_panel_width = normalized_width
        save_width = getattr(self, "save_clipboard_panel_width", None)
        if callable(save_width):
            save_width(normalized_width)

    def _apply_clipboard_panel_width(self, width: int) -> None:
        """Apply a persisted user shelf width without enabling automatic resizing."""

        dock = getattr(self, "clipboard_dock", None)
        panel = getattr(self, "clipboard_panel", None)
        normalized_width = clamp_clipboard_dock_width(width)

        if dock is None and panel is None:
            return

        if panel is not None:
            panel.resize(normalized_width, panel.height())
        if dock is not None:
            dock.resize(normalized_width, dock.height())
            resize_docks = getattr(self, "resizeDocks", None)
            if callable(resize_docks):
                resize_docks([dock], [normalized_width], Qt.Horizontal)
        self._clipboard_panel_width = normalized_width

    def _restore_clipboard_panel_width(self) -> None:
        """Restore the persisted shelf width after the dock becomes visible."""

        target_width = getattr(self, "_clipboard_panel_width", None)
        if target_width is None:
            return
        self._apply_clipboard_panel_width(target_width)

    def fit_clipboard_view(self) -> None:
        """Center and zoom the shelf canvas so all stored items stay visible."""

        canvas = getattr(self, "clipboard_canvas", None)
        if canvas is None:
            return

        canvas.resetTransform()
        bounds = self._visible_clipboard_scene_rect()
        if bounds is None:
            canvas.scene.setSceneRect(-220, -160, 440, 320)
            canvas.centerOn(0, 0)
            return

        margin = 12.0
        padded = bounds.adjusted(-margin, -margin, margin, margin)
        canvas.scene.setSceneRect(padded)
        canvas.fitInView(padded, Qt.KeepAspectRatio)
        canvas.centerOn(bounds.center())

    def _hide_clipboard_connection_ports(self) -> None:
        """Hide transition ports in the shelf so stored items stay visually compact."""

        for item in self.clipboard_state_nodes.values():
            if hasattr(item, "connection_port"):
                item.connection_port.setVisible(False)

    def fit_clipboard_panel_to_content(self) -> None:
        """Recenter and zoom the shelf view without changing the dock width."""

        self.fit_clipboard_view()
        self.statusBar().showMessage("Shelf view fitted to content", 1500)

    def refresh_clipboard_canvas(self) -> None:
        render_container_scene(
            self.clipboard_model, self._build_clipboard_scene_context()
        )
        self._hide_clipboard_connection_ports()
        self.fit_clipboard_view()
        self.update_editor_action_states()

    def _commit_current_container_change(self, *, fit_view: bool = False) -> None:
        self.render_current_container(fit_view=fit_view)
        self.record_history_checkpoint()

    def toggle_clipboard_panel(self, visible: bool | None = None) -> None:
        dock = getattr(self, "clipboard_dock", None)
        panel = getattr(self, "clipboard_panel", None)
        if dock is None and panel is None:
            return
        current_visible = dock.isVisible() if dock is not None else panel.isVisible()
        new_visible = (not current_visible) if visible is None else bool(visible)
        if not new_visible:
            self._remember_clipboard_panel_width()
        if dock is not None:
            dock.setVisible(new_visible)
            if new_visible:
                dock.raise_()
                QTimer.singleShot(0, self._restore_clipboard_panel_width)
                QTimer.singleShot(0, self.fit_clipboard_view)
        elif panel is not None:
            panel.setVisible(new_visible)
        if hasattr(self, "clipboard_panel_toggle_button"):
            self.clipboard_panel_toggle_button.setText("Hide" if new_visible else "Show")
        if hasattr(self, "canvas_clipboard_toggle_button"):
            self.canvas_clipboard_toggle_button.setText(
                "Hide Shelf" if new_visible else "Show Shelf"
            )
        if hasattr(self, "toggle_clipboard_action"):
            self.toggle_clipboard_action.setChecked(new_visible)

    def clear_clipboard_contents(self) -> None:
        self.clipboard_model = create_clipboard_container(
            get_container_kind(self.clipboard_model)
        )
        self.refresh_clipboard_canvas()
        self._remember_clipboard_panel_width()
        self.statusBar().showMessage("Shelf cleared", 2000)

    def _widget_contains_global_pos(
        self,
        widget: QWidget | None,
        global_pos: QPoint,
    ) -> bool:
        if widget is None or not widget.isVisible():
            return False
        return widget.rect().contains(widget.mapFromGlobal(global_pos))

    def _clipboard_anchor_from_global(self, global_pos: QPoint) -> QPointF:
        viewport = self.clipboard_canvas.viewport()
        if self._widget_contains_global_pos(viewport, global_pos):
            return self.clipboard_canvas.mapToScene(
                self.clipboard_canvas.mapFromGlobal(global_pos)
            )
        return self.clipboard_canvas.get_preferred_placement_scene_pos()

    def _main_canvas_anchor_from_global(self, global_pos: QPoint) -> QPointF:
        viewport = self.canvas.viewport()
        if self._widget_contains_global_pos(viewport, global_pos):
            return self.canvas.mapToScene(self.canvas.mapFromGlobal(global_pos))
        return self.canvas.get_preferred_placement_scene_pos()

    def _store_bundle_in_clipboard_model(self, bundle, anchor: QPointF) -> bool:
        if bundle.is_empty:
            return False
        if not self._ensure_clipboard_model_kind(bundle.source_kind):
            return False
        self.sync_clipboard_layout()
        paste_bundle_into_model(
            self.clipboard_model,
            bundle,
            float(anchor.x()),
            float(anchor.y()),
        )
        self.refresh_clipboard_canvas()
        return True

    def _remove_selected_clipboard_items(
        self,
        *,
        fallback_to_all: bool = False,
    ) -> None:
        selected_items = self.clipboard_canvas.scene.selectedItems()
        if fallback_to_all and not selected_items:
            selected_items = (
                list(self.clipboard_state_nodes.values())
                + list(self.clipboard_final_outcomes.values())
                + list(self.clipboard_text_blocks)
            )
        selection = collect_scene_selection(selected_items)
        self._remove_scene_selection_from_model(self.clipboard_model, selection)
        self.refresh_clipboard_canvas()

    def _paste_bundle_to_current_container(
        self,
        bundle,
        *,
        anchor: QPointF,
        remove_from_clipboard: bool,
    ) -> bool:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return False
        if bundle.is_empty:
            return False
        if not self._confirm_cross_container_paste(bundle):
            return False
        paste_bundle_into_model(
            self.current_container_model,
            bundle,
            float(anchor.x()),
            float(anchor.y()),
        )
        self._commit_current_container_change(fit_view=False)
        if remove_from_clipboard:
            self._remove_selected_clipboard_items(fallback_to_all=False)
        return True

    def handle_canvas_external_drop(self, source_canvas, global_pos: QPoint) -> bool:
        if source_canvas is self.canvas:
            if not self._widget_contains_global_pos(
                getattr(self, "clipboard_panel", None),
                global_pos,
            ):
                return False
            if self.is_read_only_mode():
                self._show_read_only_message()
                return False
            selection = self._current_scene_selection()
            bundle = self._bundle_from_scene_selection(
                self.current_container_model,
                selection,
            )
            if bundle.is_empty:
                return False
            if not self._store_bundle_in_clipboard_model(
                bundle,
                self._clipboard_anchor_from_global(global_pos),
            ):
                return False
            self._remove_scene_selection_from_model(
                self.current_container_model,
                selection,
            )
            self._commit_current_container_change(fit_view=False)
            self.statusBar().showMessage("Stored selection on the shelf", 2000)
            return True
        if source_canvas is self.clipboard_canvas:
            if not self._widget_contains_global_pos(self.canvas.viewport(), global_pos):
                return False
            bundle = self._collect_clipboard_selection_bundle(fallback_to_all=False)
            if not self._paste_bundle_to_current_container(
                bundle,
                anchor=self._main_canvas_anchor_from_global(global_pos),
                remove_from_clipboard=True,
            ):
                return False
            self.statusBar().showMessage("Restored selection from the shelf", 2000)
            return True
        return False

    def _start_pending_bundle_placement(
        self,
        bundle,
        *,
        verb: str,
        status_text: str,
    ) -> None:
        """Attach a copied selection preview to the cursor for manual placement."""

        if bundle.is_empty:
            self._reset_pending_selection_state()
            QMessageBox.warning(self, "Selection", "Please select at least one item.")
            return
        preview = build_selection_preview(bundle, verb=verb)
        self.pending_selection_bundle = bundle.clone()
        self.pending_selection_status_text = status_text
        existing_pending = self.canvas.pending_placement_item
        if existing_pending is not None:
            self.cancel_pending_node_placement(existing_pending)
        self.canvas.start_pending_bundle_placement(
            label=preview.label,
            width=preview.width,
            height=preview.height,
        )
        self.canvas.setFocus(Qt.ShortcutFocusReason)

    def start_copy_selection_placement(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        self._start_pending_bundle_placement(
            self._collect_main_selection_bundle(),
            verb="Place copy of",
            status_text="Placed copied selection",
        )

    def finalize_pending_selection_placement(self, scene_pos: QPointF) -> None:
        bundle = getattr(self, "pending_selection_bundle", None)
        if bundle is None or bundle.is_empty:
            self.cancel_pending_selection_placement()
            return
        paste_bundle_into_model(
            self.current_container_model,
            bundle,
            float(scene_pos.x()),
            float(scene_pos.y()),
        )
        self.pending_selection_bundle = None
        self.canvas.clear_pending_placement()
        self._commit_current_container_change(fit_view=False)
        self.statusBar().showMessage(
            getattr(self, "pending_selection_status_text", "Placed selection"),
            2000,
        )
        self.pending_selection_status_text = ""

    def cancel_pending_selection_placement(self) -> None:
        self.pending_selection_bundle = None
        self.pending_selection_status_text = ""
        self.canvas.clear_pending_placement()
        self.statusBar().showMessage("Placement canceled", 2000)

    def _reset_pending_selection_state(self) -> None:
        """Clear manual placement state without showing status messages."""

        self.pending_selection_bundle = None
        self.pending_selection_status_text = ""
        if hasattr(self, "canvas") and self.canvas is not None:
            self.canvas.clear_pending_placement()

    def extract_selected_items(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        selection = self._current_scene_selection()
        bundle = self._bundle_from_scene_selection(
            self.current_container_model,
            selection,
        )
        if not bundle.states:
            QMessageBox.warning(
                self,
                "Selection",
                "Please select at least one state to extract.",
            )
            return
        container_name, ok = QInputDialog.getText(
            self,
            "Extract Selection",
            "Name for the extracted state machine:",
        )
        container_name = container_name.strip()
        if not ok or not container_name:
            return
        if self.has_state_name_conflict(container_name):
            QMessageBox.warning(
                self,
                "Error",
                (
                    f"State '{container_name}' conflicts with an existing state "
                    "or final outcome in this container!"
                ),
            )
            return
        extracted_model = StateMachine(name=container_name)
        if bundle.outcomes:
            for outcome_name in bundle.outcomes:
                extracted_model.add_outcome(Outcome(name=outcome_name))
        else:
            extracted_model.add_outcome(Outcome(name="done"))
        paste_bundle_into_model(extracted_model, bundle, 0.0, 0.0)
        min_x, min_y, _max_x, _max_y = get_bundle_bounds(bundle)
        self._remove_scene_selection_from_model(self.current_container_model, selection)
        self.current_container_model.add_state(extracted_model)
        self.current_container_model.layout.set_state_position(
            extracted_model.name,
            min_x,
            min_y,
        )
        if len(self.current_container_model.states) == 1 and isinstance(
            self.current_container_model, StateMachine
        ):
            self.current_container_model.start_state = extracted_model.name
        self._commit_current_container_change(fit_view=False)
        self.statusBar().showMessage(
            f"Extracted selection into '{container_name}'",
            2000,
        )
