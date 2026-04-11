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
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtWidgets import QMenu, QMessageBox

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.runtime_breakpoints import (
    breakpoint_parent_path,
    breakpoint_tooltip,
    state_breakpoint_path,
    toggle_breakpoint_before,
)
from yasmin_editor.editor_gui.runtime_shell_context import (
    build_runtime_shell_context_payload,
    runtime_shell_allowed,
    runtime_shell_command_result,
    runtime_shell_where_text,
)
from yasmin_editor.editor_gui.runtime_state import (
    build_runtime_view_state,
    current_runtime_container_path,
    local_runtime_transition,
    normalize_runtime_path,
    root_final_transition,
    runtime_button_states,
    runtime_state_name_for_container,
)
from yasmin_editor.editor_gui.runtime_ui import (
    format_runtime_log_entry,
    runtime_canvas_frame_style,
    runtime_log_view_style,
    runtime_mode_button_style,
    runtime_status_badge_colors,
    runtime_status_badge_style,
    runtime_toggle_button_style,
)
from yasmin_editor.runtime import Runtime
from yasmin_editor.runtime.interactive_shell import InteractiveShellManager


class EditorRuntimeMixin:
    """Mixin for editor functionality split from the main window."""

    def _connect_runtime_signals(self) -> None:
        if self.runtime is None:
            return
        self.runtime.ready_changed.connect(self.update_runtime_actions)
        self.runtime.running_changed.connect(self.update_runtime_actions)
        self.runtime.blocked_changed.connect(self.update_runtime_actions)
        self.runtime.active_state_changed.connect(self.on_runtime_active_state_changed)
        self.runtime.active_transition_changed.connect(self.on_runtime_transition_changed)
        self.runtime.outcome_changed.connect(self.on_runtime_outcome_changed)
        self.runtime.status_changed.connect(self.on_runtime_status_changed)
        self.runtime.error_occurred.connect(self.on_runtime_error)
        self.runtime.log_cleared.connect(self.clear_runtime_log_view)
        self.runtime.log_appended.connect(self.append_runtime_log)

    def _create_runtime(self) -> None:
        """Create a fresh runtime instance and re-apply UI settings."""
        self.runtime = Runtime()
        self._connect_runtime_signals()
        self._refresh_runtime_shell_context()

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
        self._close_runtime_shell()
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
        active_path = normalize_runtime_path(self.runtime_active_path)
        if not active_path:
            return None

        return root_final_transition(
            root_state_name=str(active_path[0]),
            transitions_by_state=self.root_model.transitions,
            outcome=outcome,
        )

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

    def _set_runtime_toggle_button_checked(
        self,
        button_name: str,
        checked: bool,
        checked_text: str,
        unchecked_text: str,
    ) -> None:
        button = getattr(self, button_name, None)
        if button is None:
            return

        button.blockSignals(True)
        button.setChecked(checked)
        button.setText(checked_text if checked else unchecked_text)

        button.setStyleSheet(runtime_toggle_button_style(checked))

        button.blockSignals(False)

    def _set_runtime_auto_follow_button_checked(self, checked: bool) -> None:
        self._set_runtime_toggle_button_checked(
            "runtime_auto_follow_button",
            checked,
            "Auto Follow: ON",
            "Auto Follow: OFF",
        )

    def _set_runtime_cancel_sm_button_checked(self, checked: bool) -> None:
        button = getattr(self, "runtime_cancel_sm_button", None)
        if button is None:
            return

        button.blockSignals(True)
        button.setChecked(False)
        button.setText("Cancel State Machine")
        button.blockSignals(False)

    def _ensure_runtime_shell(self) -> Optional[InteractiveShellManager]:
        shell = getattr(self, "runtime_shell", None)
        if shell is not None:
            return shell

        if not InteractiveShellManager.is_supported():
            return None

        self.runtime_shell = InteractiveShellManager(self)
        self.runtime_shell.visibility_changed.connect(
            self.on_runtime_shell_visibility_changed
        )
        return self.runtime_shell

    def _runtime_shell_command_result(self, command_name: str) -> str:
        return runtime_shell_command_result(self.runtime, command_name)

    def _runtime_shell_where(self) -> str:
        return runtime_shell_where_text(self.runtime)

    def _build_runtime_shell_commands(self) -> dict[str, object]:
        return {
            "next": lambda: (
                self.on_runtime_step_clicked(),
                self._runtime_shell_command_result("next"),
            )[1],
            "step": lambda: (
                self.on_runtime_step_clicked(),
                self._runtime_shell_command_result("step"),
            )[1],
            "cont": lambda: (
                self.on_runtime_play_clicked(),
                self._runtime_shell_command_result("cont"),
            )[1],
            "play": lambda: (
                self.on_runtime_play_clicked(),
                self._runtime_shell_command_result("play"),
            )[1],
            "pause": lambda: (
                self.on_runtime_pause_clicked(),
                self._runtime_shell_command_result("pause"),
            )[1],
            "cancel_state": lambda: (
                self.on_runtime_cancel_state_clicked(),
                self._runtime_shell_command_result("cancel_state"),
            )[1],
            "cancel_sm": lambda: (
                self.on_runtime_cancel_sm_clicked(),
                self._runtime_shell_command_result("cancel_sm"),
            )[1],
            "restart": lambda: (
                self.restart_runtime_mode(),
                self._runtime_shell_command_result("restart"),
            )[1],
            "where": self._runtime_shell_where,
        }

    def _refresh_runtime_shell_context(self) -> None:
        shell = getattr(self, "runtime_shell", None)
        if shell is None:
            return

        payload = build_runtime_shell_context_payload(
            self.runtime,
            self._build_runtime_shell_commands(),
        )
        if payload is None:
            return

        shell.update_context(**payload)

    def _close_runtime_shell(self) -> None:
        shell = getattr(self, "runtime_shell", None)
        if shell is not None:
            shell.close_shell()

    def _shutdown_runtime_shell(self) -> None:
        shell = getattr(self, "runtime_shell", None)
        if shell is not None:
            shell.shutdown()
        self.runtime_shell = None

    def _is_runtime_shell_open(self) -> bool:
        shell = getattr(self, "runtime_shell", None)
        return bool(shell is not None and shell.is_open())

    def _runtime_shell_execution_blocked(self) -> bool:
        return False

    def _runtime_requires_finish_before_leaving(self) -> bool:
        runtime = self.runtime
        return bool(
            self.runtime_mode_enabled
            and runtime is not None
            and runtime.is_running()
            and not runtime.is_finished()
        )

    def _normalize_runtime_breakpoint_path(
        self, path: tuple[str, ...]
    ) -> tuple[str, ...]:
        return normalize_runtime_path(path)

    def _current_runtime_breakpoint_parent_path(self) -> tuple[str, ...]:
        return breakpoint_parent_path(
            self.runtime_mode_enabled,
            self._get_current_runtime_container_path(),
        )

    def _state_node_runtime_breakpoint_path(self, state_node) -> tuple[str, ...]:
        return state_breakpoint_path(
            self.runtime_mode_enabled,
            self._get_current_runtime_container_path(),
            getattr(state_node, "name", ""),
        )

    def _runtime_breakpoint_marker_tooltip(self, state_path: tuple[str, ...]) -> str:
        return breakpoint_tooltip(state_path, self.runtime_breakpoints_before)

    def update_runtime_breakpoint_markers(self) -> None:
        for state_node in list(self.state_nodes.values()):
            if self._is_deleted_graphics_item(state_node):
                continue
            marker = getattr(state_node, "set_breakpoint_marker", None)
            if marker is None:
                continue
            state_path = self._state_node_runtime_breakpoint_path(state_node)
            has_breakpoint = state_path in self.runtime_breakpoints_before
            marker(has_breakpoint, self._runtime_breakpoint_marker_tooltip(state_path))

    def _sync_runtime_breakpoints_to_backend(self) -> None:
        runtime = self.runtime
        if runtime is None:
            return
        runtime.set_breakpoints(
            before_paths=sorted(self.runtime_breakpoints_before),
        )

    def _toggle_runtime_breakpoint(self, state_node) -> None:
        state_path = self._state_node_runtime_breakpoint_path(state_node)
        updated_paths, action_label = toggle_breakpoint_before(
            self.runtime_breakpoints_before,
            state_path,
        )
        if not action_label:
            return

        self.runtime_breakpoints_before = updated_paths
        self._sync_runtime_breakpoints_to_backend()
        self.refresh_visual_highlighting()
        self.statusBar().showMessage(
            f"Breakpoint {action_label}: {' / '.join(state_path)}",
            3000,
        )

    def show_runtime_breakpoint_menu(
        self,
        state_node,
        global_pos,
    ) -> bool:
        if not self.runtime_mode_enabled:
            return False

        state_path = self._state_node_runtime_breakpoint_path(state_node)
        if not state_path:
            return False

        menu = QMenu(self)
        before_enabled = state_path in self.runtime_breakpoints_before

        toggle_action = menu.addAction(
            "Remove Breakpoint" if before_enabled else "Add Breakpoint"
        )

        action = menu.exec_(global_pos)
        if action == toggle_action:
            self._toggle_runtime_breakpoint(state_node)
            return True
        return False

    def _show_runtime_finish_required_popup(self) -> None:
        QMessageBox.information(
            self,
            "Runtime Still Running",
            "The state machine is still running. Cancel the state machine and "
            "wait until it finishes before leaving runtime mode.",
        )

    def _enter_runtime_mode(self) -> bool:
        self._close_runtime_shell()
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
        self._sync_runtime_breakpoints_to_backend()
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
        self._refresh_runtime_shell_context()

    def on_runtime_pause_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.pause()
        self._refresh_runtime_shell_context()

    def on_runtime_step_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.play_once()
        self._refresh_runtime_shell_context()

    def on_runtime_cancel_state_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.cancel_state()
        self._refresh_runtime_shell_context()

    def on_runtime_cancel_sm_clicked(self) -> None:
        runtime = self.runtime
        if runtime is None:
            return

        runtime.cancel_state_machine()
        self._refresh_runtime_shell_context()
        self.update_runtime_actions()

    def restart_runtime_mode(self) -> None:
        runtime = self.runtime
        if not self.runtime_mode_enabled or runtime is None or not runtime.is_finished():
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

    def on_runtime_shell_clicked(self) -> None:
        runtime = self.runtime
        if not runtime_shell_allowed(runtime):
            self.statusBar().showMessage(
                "The interactive shell is available while runtime mode is active and a runtime state machine is loaded.",
                3000,
            )
            return

        shell = self._ensure_runtime_shell()
        if shell is None:
            QMessageBox.critical(
                self,
                "Interactive Shell Unavailable",
                "The interactive shell requires qtconsole.\n\n"
                "Install it with:\n"
                "python3 -m pip install qtconsole\n\n"
                f"Import error: {InteractiveShellManager.unavailable_reason()}",
            )
            return

        payload = build_runtime_shell_context_payload(
            runtime,
            self._build_runtime_shell_commands(),
        )
        if payload is None:
            return

        shell.open_shell(**payload)
        self.update_runtime_actions()

    def on_runtime_shell_visibility_changed(self, visible: bool) -> None:
        if visible:
            self.statusBar().showMessage(
                "Interactive shell opened. Recommendation: avoid modifying the blackboard while the runtime is running.",
                5000,
            )
        else:
            self.statusBar().showMessage("Interactive shell closed", 3000)
        self.update_runtime_actions()

    def toggle_runtime_mode(self, checked: bool) -> None:
        if checked:
            self._enter_runtime_mode()
            return

        if self._runtime_requires_finish_before_leaving():
            self._set_runtime_mode_button_checked(True)
            self._show_runtime_finish_required_popup()
            return

        self._close_runtime_shell()
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
            if self.runtime_mode_enabled and self.runtime is not None:
                status = self.runtime.get_status_label()
                border_color = self._runtime_status_badge_colors(status)[0]
            else:
                border_color = PALETTE.ui_border
            self.canvas_frame.setStyleSheet(
                runtime_canvas_frame_style(border_width, border_color)
            )

        if hasattr(self, "runtime_mode_button"):
            self.runtime_mode_button.setText(
                "Runtime Mode Active" if self.runtime_mode_enabled else "Runtime Mode"
            )
            self.runtime_mode_button.setStyleSheet(
                runtime_mode_button_style(self.runtime_mode_enabled)
            )

    def _runtime_status_badge_colors(self, status: str):
        """Return background, foreground and border colors for the runtime badge."""
        return runtime_status_badge_colors(status)

    def _runtime_status_badge_style(self, status: str) -> str:
        """Return the stylesheet for the runtime status badge."""
        return runtime_status_badge_style(status)

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
        return runtime_log_view_style()

    def _format_runtime_log_entry(self, message: str) -> str:
        """Convert a runtime log message into a styled HTML block."""
        return format_runtime_log_entry(message)

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
        self._refresh_runtime_shell_context()
        self._follow_runtime_active_state()
        self._schedule_runtime_highlight_refresh()
        self.update_runtime_actions()

    def on_runtime_transition_changed(
        self,
        transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    ) -> None:
        self.runtime_last_transition = transition
        self._refresh_runtime_shell_context()
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
        self._refresh_runtime_shell_context()
        self.update_runtime_actions()

    def on_runtime_error(self, message: str) -> None:
        self.statusBar().showMessage("Runtime error", 3000)
        self.append_runtime_log(f"[ERROR] {message}")
        self._update_runtime_status_badge("Error")
        QMessageBox.critical(self, "Runtime Error", message)
        self.update_runtime_actions()

    def update_runtime_actions(self) -> None:
        runtime = self.runtime
        runtime_state = build_runtime_view_state(
            runtime_mode_enabled=self.runtime_mode_enabled,
            runtime=runtime,
            shell_execution_blocked=self._runtime_shell_execution_blocked(),
        )

        self._set_runtime_mode_button_checked(self.runtime_mode_enabled)
        self.set_canvas_runtime_visual_state()

        if hasattr(self, "runtime_status_label"):
            self._update_runtime_status_badge(
                runtime.get_status_label() if runtime is not None else "Inactive"
            )

        if hasattr(self, "runtime_log_view"):
            self.runtime_log_view.setStyleSheet(self._runtime_log_view_style())

        self._sync_runtime_log_level_combo()

        for button_name, button_state in runtime_button_states(runtime_state).items():
            button = getattr(self, button_name, None)
            if button is None:
                continue
            button.setVisible(button_state.visible)
            button.setEnabled(button_state.enabled)

        self._set_runtime_cancel_sm_button_checked(False)
        self._set_runtime_auto_follow_button_checked(self.runtime_auto_follow_enabled)

        for action_name in [
            "new_action",
            "open_action",
        ]:
            action = getattr(self, action_name, None)
            if action is not None:
                action.setEnabled(not self.runtime_mode_enabled)

        self.update_editor_action_states()

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
        return current_runtime_container_path(
            explicit_path=self.current_runtime_container_path,
            current_container_path=self.current_container_path,
        )

    def _get_runtime_state_name_for_current_container(self) -> Optional[str]:
        if not self.runtime_mode_enabled:
            return None

        return runtime_state_name_for_container(
            active_path=self._get_live_runtime_active_path(),
            current_path=self._get_current_runtime_container_path(),
        )

    def _find_runtime_connection_for_current_container(
        self,
    ) -> Optional[ConnectionLine]:
        if not self.runtime_mode_enabled:
            return None

        local_transition = local_runtime_transition(
            transition=self._get_live_runtime_transition(),
            current_path=self._get_current_runtime_container_path(),
        )
        if local_transition is None:
            return None

        for connection in list(self.connections):
            if self._is_deleted_connection_item(connection):
                continue
            if (
                getattr(connection.from_node, "name", None) == local_transition.from_name
                and getattr(connection.to_node, "name", None) == local_transition.to_name
                and connection.outcome == local_transition.outcome
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
