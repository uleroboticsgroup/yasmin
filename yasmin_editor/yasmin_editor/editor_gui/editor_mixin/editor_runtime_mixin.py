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

import os
import tempfile
from html import escape
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QPen
from PyQt5.QtWidgets import QMessageBox

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.nodes.container_state_node import \
    ContainerStateNode
from yasmin_editor.runtime import Runtime


class EditorRuntimeMixin:
    """Mixin for editor functionality split from the main window."""

    def _connect_runtime_signals(self) -> None:
        if self.runtime is None:
            return
        self.runtime.ready_changed.connect(self.update_runtime_actions)
        self.runtime.running_changed.connect(self.update_runtime_actions)
        self.runtime.blocked_changed.connect(self.update_runtime_actions)
        self.runtime.active_state_changed.connect(self.on_runtime_active_state_changed)
        self.runtime.active_transition_changed.connect(
            self.on_runtime_transition_changed
        )
        self.runtime.outcome_changed.connect(self.on_runtime_outcome_changed)
        self.runtime.status_changed.connect(self.on_runtime_status_changed)
        self.runtime.error_occurred.connect(self.on_runtime_error)
        self.runtime.log_cleared.connect(self.clear_runtime_log_view)
        self.runtime.log_appended.connect(self.append_runtime_log)

    def _create_runtime(self) -> None:
        """Create a fresh runtime instance and re-apply UI settings."""
        self.runtime = Runtime()
        self._connect_runtime_signals()

        if hasattr(self, "runtime_log_level_combo"):
            try:
                self.runtime.set_log_level(
                    self.runtime_log_level_combo.currentText(),
                    emit_status=False,
                )
            except Exception:
                pass

        self._sync_runtime_log_level_combo()

    def _destroy_runtime(self) -> None:
        runtime = self.runtime
        self.runtime = None
        if runtime is None:
            return
        try:
            runtime.disconnect()
        except Exception:
            pass
        try:
            runtime.shutdown()
        except Exception:
            pass
        try:
            runtime.deleteLater()
        except Exception:
            pass
        del runtime

    def _recreate_runtime(self) -> None:
        self._destroy_runtime()
        self._create_runtime()

    def _delete_runtime_snapshot(self) -> None:
        if self.runtime_snapshot_file_path and os.path.isfile(
            self.runtime_snapshot_file_path
        ):
            try:
                os.remove(self.runtime_snapshot_file_path)
            except OSError:
                pass
        self.runtime_snapshot_file_path = None

    def _runtime_ready(self) -> bool:
        return self.runtime is not None

    def _get_live_runtime_active_path(self) -> tuple[str, ...]:
        if self.runtime is None or not self.runtime_mode_enabled:
            return tuple()
        try:
            active_path = tuple(self.runtime.get_active_path() or tuple())
        except Exception:
            active_path = tuple()
        self.runtime_active_path = active_path
        return active_path

    def _get_live_runtime_transition(
        self,
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        if self.runtime is None or not self.runtime_mode_enabled:
            return None

        try:
            transition = self.runtime.get_last_transition()
        except Exception:
            transition = None

        if transition is None and self.runtime.is_finished():
            return self.runtime_last_transition

        self.runtime_last_transition = transition
        return transition

    def _build_root_final_transition(
        self,
        outcome: Optional[str],
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        if not outcome:
            return None

        active_path = tuple(self.runtime_active_path or tuple())
        if not active_path:
            return None

        root_state_name = str(active_path[0])
        transitions = self.root_model.transitions.get(root_state_name, [])

        for transition in transitions:
            if str(transition.target) == str(outcome):
                return (
                    (root_state_name,),
                    (str(outcome),),
                    str(transition.source_outcome),
                )

        return None

    def _schedule_runtime_highlight_refresh(self) -> None:
        if not self.runtime_mode_enabled:
            return
        QTimer.singleShot(0, self._refresh_runtime_highlighting_from_runtime)

    def _refresh_runtime_highlighting_from_runtime(self) -> None:
        if not self.runtime_mode_enabled:
            return
        self._get_live_runtime_active_path()
        self._get_live_runtime_transition()
        self.refresh_visual_highlighting()

    def create_runtime_xml_snapshot(self) -> str:
        self.sync_current_container_layout()
        self._delete_runtime_snapshot()

        fd, temp_path = tempfile.mkstemp(
            prefix="yasmin_editor_runtime_",
            suffix=".xml",
        )
        os.close(fd)
        self.runtime_snapshot_file_path = temp_path

        self.save_to_xml(self.runtime_snapshot_file_path)
        return self.runtime_snapshot_file_path

    def _set_runtime_mode_button_checked(self, checked: bool) -> None:
        if not hasattr(self, "runtime_mode_button"):
            return
        self.runtime_mode_button.blockSignals(True)
        self.runtime_mode_button.setChecked(checked)
        self.runtime_mode_button.blockSignals(False)

    def _set_runtime_auto_follow_button_checked(self, checked: bool) -> None:
        if not hasattr(self, "runtime_auto_follow_button"):
            return

        button = self.runtime_auto_follow_button
        button.blockSignals(True)
        button.setChecked(checked)
        button.setText("Auto Follow: ON" if checked else "Auto Follow: OFF")

        if checked:
            button.setStyleSheet(
                "QPushButton {"
                f"background-color: {PALETTE.runtime_mode_button_bg.name()}; "
                f"color: {PALETTE.runtime_mode_button_text.name()}; "
                f"border: 2px solid {PALETTE.runtime_canvas_border.name()}; "
                "font-weight: 700;"
                "padding: 4px 10px;"
                "}"
            )
        else:
            button.setStyleSheet(
                "QPushButton {"
                f"background-color: {PALETTE.ui_button_bg.name()}; "
                f"color: {PALETTE.text_primary.name()}; "
                f"border: 2px solid {PALETTE.ui_border.name()}; "
                "font-weight: 700;"
                "padding: 4px 10px;"
                "}"
            )

        button.blockSignals(False)

    def _enter_runtime_mode(self) -> bool:
        self._recreate_runtime()

        try:
            runtime_path = self.create_runtime_xml_snapshot()
        except Exception as exc:
            self.runtime_mode_enabled = False
            self._set_runtime_mode_button_checked(False)
            QMessageBox.critical(
                self,
                "Runtime Error",
                f"Failed to enter runtime mode:\n{exc}",
            )
            self.update_runtime_actions()
            return False

        if self.runtime is None or not self.runtime.create_sm_from_file(runtime_path):
            self.runtime_mode_enabled = False
            self._set_runtime_mode_button_checked(False)
            self.update_runtime_actions()
            return False

        self.runtime_mode_enabled = True
        self._get_live_runtime_active_path()
        self._get_live_runtime_transition()
        self.render_current_container()
        self._follow_runtime_active_state()
        self.set_canvas_runtime_visual_state()
        self.statusBar().showMessage("Runtime mode enabled", 3000)
        return True

    def on_runtime_play_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.play()

    def on_runtime_pause_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.pause()

    def on_runtime_step_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.play_once()

    def on_runtime_cancel_state_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.cancel_state()

    def on_runtime_cancel_sm_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.cancel()

    def restart_runtime_mode(self) -> None:
        if not self.runtime_mode_enabled:
            return

        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self._delete_runtime_snapshot()
        self._recreate_runtime()
        self.clear_runtime_log_view()
        self._enter_runtime_mode()

    def on_runtime_auto_follow_toggled(self, checked: bool) -> None:
        self.runtime_auto_follow_enabled = bool(checked)
        self._set_runtime_auto_follow_button_checked(self.runtime_auto_follow_enabled)
        if self.runtime_auto_follow_enabled:
            self._follow_runtime_active_state()
        self.update_runtime_actions()

    def toggle_runtime_mode(self, checked: bool) -> None:
        if checked:
            self._enter_runtime_mode()
            return

        self.runtime_mode_enabled = False
        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self._delete_runtime_snapshot()
        self._recreate_runtime()
        self.clear_runtime_log_view()
        self.render_current_container()
        self.set_canvas_runtime_visual_state()
        self.statusBar().showMessage("Runtime mode disabled", 3000)

    def set_canvas_runtime_visual_state(self) -> None:
        if hasattr(self, "runtime_controls_widget"):
            self.runtime_controls_widget.setVisible(self.runtime_mode_enabled)

        if hasattr(self, "editor_sidebar_widget"):
            self.editor_sidebar_widget.setVisible(not self.runtime_mode_enabled)
        if hasattr(self, "runtime_sidebar_widget"):
            self.runtime_sidebar_widget.setVisible(self.runtime_mode_enabled)

        if hasattr(self, "canvas_frame"):
            border_width = 3 if self.runtime_mode_enabled else 1
            border_color = (
                PALETTE.runtime_canvas_border
                if self.runtime_mode_enabled
                else PALETTE.ui_border
            )
            self.canvas_frame.setStyleSheet(
                f"QFrame {{ border: {border_width}px solid {border_color.name()}; }}"
            )

        if hasattr(self, "runtime_mode_button"):
            self.runtime_mode_button.setText(
                "Runtime Mode Active" if self.runtime_mode_enabled else "Runtime Mode"
            )
            if self.runtime_mode_enabled:
                self.runtime_mode_button.setStyleSheet(
                    "QPushButton {"
                    f"background-color: {PALETTE.runtime_mode_button_bg.name()}; "
                    f"color: {PALETTE.runtime_mode_button_text.name()}; "
                    f"border: 1px solid {PALETTE.runtime_canvas_border.name()};"
                    "}"
                )
            else:
                self.runtime_mode_button.setStyleSheet("")

    def _runtime_status_badge_style(self, status: str) -> str:
        """Return the stylesheet for the runtime status badge."""
        normalized = str(status).strip().lower()

        background = QColor(205, 210, 214)
        foreground = QColor(28, 31, 36)
        border = QColor(148, 154, 160)

        if normalized == "running":
            background = QColor(46, 150, 76)
            foreground = QColor(255, 255, 255)
            border = QColor(33, 108, 54)
        elif normalized == "paused":
            background = QColor(214, 170, 52)
            foreground = QColor(24, 24, 24)
            border = QColor(150, 118, 34)
        elif normalized == "inactive":
            background = QColor(145, 149, 156)
            foreground = QColor(255, 255, 255)
            border = QColor(103, 108, 115)
        elif normalized == "ready":
            background = QColor(224, 228, 232)
            foreground = QColor(30, 34, 40)
            border = QColor(172, 178, 184)
        elif normalized in {"succeeded", "success"}:
            background = QColor(46, 150, 76)
            foreground = QColor(255, 255, 255)
            border = QColor(33, 108, 54)
        elif normalized in {"aborted", "failed", "failure", "error"}:
            background = QColor(176, 60, 60)
            foreground = QColor(255, 255, 255)
            border = QColor(120, 40, 40)

        return (
            "QLabel {"
            f"background-color: {background.name()}; "
            f"color: {foreground.name()}; "
            f"border: 1px solid {border.name()}; "
            "border-radius: 10px; "
            "padding: 4px 12px; "
            "font-weight: 600;"
            "}"
        )

    def _update_runtime_status_badge(self, status: str) -> None:
        """Apply runtime status text and styling to the badge widget."""
        if not hasattr(self, "runtime_status_label"):
            return
        label_text = str(status).strip() or "Inactive"
        self.runtime_status_label.setText(label_text)
        self.runtime_status_label.setStyleSheet(
            self._runtime_status_badge_style(label_text)
        )

    def _runtime_log_view_style(self) -> str:
        """Return the stylesheet used by the runtime log view."""
        return (
            "QTextBrowser {"
            f"background-color: {PALETTE.ui_input_bg.name()}; "
            f"color: {PALETTE.text_primary.name()}; "
            f"border: 1px solid {PALETTE.ui_border.name()}; "
            f"selection-background-color: {PALETTE.ui_selection_bg.name()}; "
            f"selection-color: {PALETTE.ui_selection_text.name()}; "
            "border-radius: 6px;"
            "}"
        )

    def _runtime_log_line_color(self, message: str) -> str:
        """Return the HTML color used for a runtime log line."""
        # split removes the tree stucture from the log
        normalized = "[" + str(message).split("[", 1)[-1].lstrip()
        if normalized.startswith("[WARN]"):
            return PALETTE.runtime_log_warn.name()
        if normalized.startswith("[ERROR]"):
            return PALETTE.runtime_log_error.name()
        if normalized.startswith("[STATUS]"):
            return PALETTE.runtime_log_status.name()
        if normalized.startswith("[INFO]"):
            return PALETTE.runtime_log_info.name()
        if normalized.startswith("[DEBUG]"):
            return PALETTE.runtime_log_debug.name()
        if normalized.startswith("[TRANSITION]"):
            return PALETTE.runtime_log_system.name()
        if normalized.startswith("[START]"):
            return PALETTE.runtime_log_system.name()
        if normalized.startswith("[END]"):
            return PALETTE.runtime_log_system.name()
        return PALETTE.runtime_log_default.name()

    def _format_runtime_log_entry(self, message: str) -> str:
        """Convert a runtime log message into a styled HTML block."""
        raw_message = str(message)
        color = self._runtime_log_line_color(raw_message)

        prefix = ""
        content = raw_message
        first_bracket_index = raw_message.find("[")
        if first_bracket_index > 0:
            prefix = raw_message[:first_bracket_index]
            content = raw_message[first_bracket_index:]

        return (
            '<div style="'
            "font-family: 'DejaVu Sans Mono', 'Courier New', monospace; "
            "white-space: pre; "
            'margin: 0;">'
            f'<span style="color: {PALETTE.text_primary.name()};">{escape(prefix)}</span>'
            f'<span style="color: {color};">{escape(content)}</span>'
            "</div>"
        )

    def _sync_runtime_log_level_combo(self) -> None:
        """Keep the runtime log-level selector aligned with the backend state."""
        if not hasattr(self, "runtime_log_level_combo"):
            return

        combo = self.runtime_log_level_combo
        runtime = self.runtime
        target_level = "INFO"
        if runtime is not None:
            try:
                target_level = runtime.get_log_level_name()
            except Exception:
                target_level = combo.currentText() or "INFO"

        combo.blockSignals(True)
        combo.setCurrentText(target_level)
        combo.blockSignals(False)

    def on_runtime_log_level_changed(self, level_name: str) -> None:
        """Apply the selected runtime log level to the active runtime backend."""
        runtime = self.runtime
        if runtime is None:
            return

        try:
            runtime.set_log_level(level_name)
        except Exception as exc:
            self.statusBar().showMessage("Failed to update log level", 3000)
            QMessageBox.critical(
                self,
                "Runtime Error",
                f"Failed to update runtime log level:\n{exc}",
            )
            self._sync_runtime_log_level_combo()

    def clear_runtime_log_view(self) -> None:
        """Clear the runtime log widget shown in the sidebar."""
        if hasattr(self, "runtime_log_view"):
            self.runtime_log_view.clear()

    def append_runtime_log(self, message: str) -> None:
        """Append a formatted runtime log message to the sidebar view."""
        if not hasattr(self, "runtime_log_view"):
            return
        self.runtime_log_view.append(self._format_runtime_log_entry(str(message)))
        scrollbar = self.runtime_log_view.verticalScrollBar()
        if scrollbar is not None:
            scrollbar.setValue(scrollbar.maximum())

    def on_runtime_active_state_changed(self, state_path: tuple[str, ...]) -> None:
        self.runtime_active_path = tuple(state_path or tuple())
        self._follow_runtime_active_state()
        self._schedule_runtime_highlight_refresh()
        self.update_runtime_actions()

    def on_runtime_transition_changed(
        self,
        transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    ) -> None:
        self.runtime_last_transition = transition
        self._schedule_runtime_highlight_refresh()

    def on_runtime_outcome_changed(self, outcome: Optional[str]) -> None:
        self._get_live_runtime_active_path()
        self.runtime_last_transition = self._build_root_final_transition(outcome)

        if self.runtime_mode_enabled:
            self.navigate_to_container_index(0)

        self.refresh_visual_highlighting()
        self.update_runtime_actions()

    def on_runtime_status_changed(self, message: str) -> None:
        self.statusBar().showMessage(message, 3000)
        self.append_runtime_log(f"[STATUS] {message}")
        runtime = self.runtime
        if runtime is not None:
            self._update_runtime_status_badge(runtime.get_status_label())
        self.update_runtime_actions()

    def on_runtime_error(self, message: str) -> None:
        self.statusBar().showMessage("Runtime error", 3000)
        self.append_runtime_log(f"[ERROR] {message}")
        self._update_runtime_status_badge("Error")
        QMessageBox.critical(self, "Runtime Error", message)
        self.update_runtime_actions()

    def update_runtime_actions(self) -> None:
        runtime = self.runtime
        runtime_ready = (
            self.runtime_mode_enabled and runtime is not None and runtime.is_ready()
        )
        runtime_running = runtime_ready and runtime.is_running()
        runtime_blocked = runtime_running and runtime.is_blocked()
        runtime_playing = runtime_running and not runtime_blocked
        runtime_step_mode = runtime_running and runtime.is_step_mode()
        runtime_finished = runtime_ready and runtime.is_finished()

        self._set_runtime_mode_button_checked(self.runtime_mode_enabled)
        self.set_canvas_runtime_visual_state()

        if hasattr(self, "runtime_status_label"):
            self._update_runtime_status_badge(
                runtime.get_status_label() if runtime is not None else "Inactive"
            )

        if hasattr(self, "runtime_log_view"):
            self.runtime_log_view.setStyleSheet(self._runtime_log_view_style())

        self._sync_runtime_log_level_combo()

        runtime_button_visibility = {
            "runtime_play_button": runtime_ready
            and not runtime_playing
            and not runtime_finished,
            "runtime_pause_button": runtime_playing and not runtime_step_mode,
            "runtime_step_button": runtime_ready
            and not runtime_playing
            and not runtime_finished,
            "runtime_cancel_state_button": runtime_running and not runtime_finished,
            "runtime_cancel_sm_button": runtime_running and not runtime_finished,
            "runtime_restart_button": runtime_ready,
        }
        for button_name, visible in runtime_button_visibility.items():
            button = getattr(self, button_name, None)
            if button is not None:
                button.setVisible(visible)
                button.setEnabled(visible)

        auto_follow_button = getattr(self, "runtime_auto_follow_button", None)
        if auto_follow_button is not None:
            auto_follow_button.setVisible(self.runtime_mode_enabled)
            auto_follow_button.setEnabled(runtime_ready)
            self._set_runtime_auto_follow_button_checked(
                self.runtime_auto_follow_enabled
            )

        for action_name in [
            "new_action",
            "open_action",
        ]:
            action = getattr(self, action_name, None)
            if action is not None:
                action.setEnabled(not self.runtime_mode_enabled)

    def _follow_runtime_active_state(self) -> None:
        if not self.runtime_mode_enabled or not self.runtime_auto_follow_enabled:
            return

        active_path = tuple(self.runtime_active_path or tuple())
        target_path = active_path[:-1] if active_path else tuple()
        self._navigate_to_runtime_container_path(target_path)

    def _navigate_to_runtime_container_path(self, target_path: tuple[str, ...]) -> None:
        if not self.runtime_mode_enabled:
            return

        normalized_target_path = tuple(str(item) for item in target_path)
        if self._get_current_runtime_container_path() == normalized_target_path:
            return

        self.navigate_to_container_index(0)

        for state_name in normalized_target_path:
            container_node = self.state_nodes.get(state_name)
            if not isinstance(container_node, ContainerStateNode):
                return
            self.enter_container(container_node, show_status_message=False)

    def _get_current_runtime_container_path(self) -> tuple[str, ...]:
        if self.current_runtime_container_path:
            return tuple(self.current_runtime_container_path)
        return tuple(
            str(container.name) for container in self.current_container_path[1:]
        )

    def _get_runtime_state_name_for_current_container(self) -> Optional[str]:
        if not self.runtime_mode_enabled:
            return None

        active_path = self._get_live_runtime_active_path()
        if not active_path:
            return None

        current_path = self._get_current_runtime_container_path()
        if len(current_path) >= len(active_path):
            return None

        if active_path[: len(current_path)] != current_path:
            return None

        return active_path[len(current_path)]

    def _find_runtime_connection_for_current_container(
        self,
    ) -> Optional[ConnectionLine]:
        transition = self._get_live_runtime_transition()
        if not self.runtime_mode_enabled or not transition:
            return None

        from_path, to_path, outcome = transition
        current_path = self._get_current_runtime_container_path()
        expected_depth = len(current_path) + 1

        if len(from_path) != expected_depth or len(to_path) != expected_depth:
            return None

        if from_path[: len(current_path)] != current_path:
            return None

        if to_path[: len(current_path)] != current_path:
            return None

        from_name = from_path[len(current_path)]
        to_name = to_path[len(current_path)]

        for connection in list(self.connections):
            if self._is_deleted_connection_item(connection):
                continue
            if (
                getattr(connection.from_node, "name", None) == from_name
                and getattr(connection.to_node, "name", None) == to_name
                and connection.outcome == outcome
            ):
                return connection

        return None

    def apply_runtime_highlighting(self) -> None:
        active_name = self._get_runtime_state_name_for_current_container()
        if not active_name:
            return

        active_item = self.state_nodes.get(active_name)
        if active_item is None or self._is_deleted_graphics_item(active_item):
            return

        try:
            active_item.setPen(QPen(PALETTE.runtime_highlight_pen, 5))
            active_item.setBrush(QBrush(PALETTE.runtime_highlight_fill))
        except RuntimeError:
            return

    def apply_runtime_transition_highlighting(self) -> None:
        connection = self._find_runtime_connection_for_current_container()
        if connection is None or self._is_deleted_connection_item(connection):
            return

        try:
            pen = QPen(PALETTE.runtime_highlight_pen, 5)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            connection.setPen(pen)
            connection.arrow_head.setBrush(QBrush(PALETTE.runtime_highlight_pen))
            connection.arrow_head.setPen(QPen(PALETTE.runtime_highlight_pen))
            connection.label_bg.setBrush(QBrush(PALETTE.runtime_highlight_fill))
            connection.label_bg.setPen(QPen(PALETTE.runtime_highlight_pen, 2))
        except RuntimeError:
            return
