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

"""Runtime execution backend for the YASMIN editor.

The runtime keeps a live state machine instance, mirrors its execution state into
Qt signals, and provides a small control surface for play, pause, step, cancel,
and shutdown.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Iterable, Optional

from PyQt5.QtCore import QObject, pyqtSignal
from yasmin_editor.runtime.traversal import (container_states,
                                             expand_to_deepest_known_path,
                                             is_container_object,
                                             resolve_container)

import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_factory import YasminFactory

LOG_LEVEL_BY_NAME = {
    "ERROR": yasmin.LogLevel.ERROR,
    "WARN": yasmin.LogLevel.WARN,
    "INFO": yasmin.LogLevel.INFO,
    "DEBUG": yasmin.LogLevel.DEBUG,
}


class Runtime(QObject):
    """Execute and observe a YASMIN state machine instance.

    The class exposes Qt signals so the editor can react to state changes,
    transitions, status updates, and log output without polling.
    """

    ready_changed = pyqtSignal(bool)
    running_changed = pyqtSignal(bool)
    blocked_changed = pyqtSignal(bool)
    active_state_changed = pyqtSignal(object)
    active_transition_changed = pyqtSignal(object)
    outcome_changed = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    log_cleared = pyqtSignal()
    log_appended = pyqtSignal(str)

    def __init__(self) -> None:
        """Initialize the runtime controller and its logger bridge."""
        super().__init__()

        self.factory = YasminFactory()
        self.sm: Optional[StateMachine] = None
        self.bb: Optional[Blackboard] = None

        self._running = False
        self._blocked = False
        self._finished = False
        self._final_outcome: Optional[str] = None
        self._disposed = False
        self._shutting_down = False

        self._active_path: tuple[str, ...] = tuple()
        self._last_transition: Optional[
            tuple[tuple[str, ...], tuple[str, ...], str]
        ] = None

        self._execution_thread: Optional[threading.Thread] = None
        self._worker_state_lock = threading.Lock()
        self._pause_condition = threading.Condition()
        self._pause_requested = False

        self._step_mode = False

        self._log_entries: list[str] = []
        self._log_buffer_lock = threading.Lock()
        self._last_log_message = ""
        self._last_log_timestamp = 0.0
        self._log_level = yasmin.LogLevel.INFO

        self._configure_logging()

    def is_ready(self) -> bool:
        """Return whether a state machine is currently loaded."""
        return isinstance(self.sm, StateMachine)

    def is_running(self) -> bool:
        """Return whether the runtime worker is actively executing."""
        return self._running

    def is_blocked(self) -> bool:
        """Return whether execution is currently paused."""
        return self._blocked

    def is_finished(self) -> bool:
        """Return whether the loaded state machine finished execution."""
        return self._finished

    def is_step_mode(self) -> bool:
        """Return whether execution is currently armed for a single step."""
        return self._step_mode

    def get_final_outcome(self) -> Optional[str]:
        """Return the final outcome once execution completed."""
        return self._final_outcome

    def get_status_label(self) -> str:
        """Return the human-readable runtime status shown in the UI."""
        if self._finished and self._final_outcome:
            return self._final_outcome
        if self._running and self._blocked:
            return "Paused"
        if self._running:
            return "Running"
        if self.is_ready():
            return "Ready"
        return "Inactive"

    def get_current_state(self) -> Optional[str]:
        """Return the currently active leaf state name if available."""
        return self._active_path[-1] if self._active_path else None

    def get_active_path(self) -> tuple[str, ...]:
        """Return the last known active path inside the loaded machine."""
        return self._active_path

    def get_live_active_path(self) -> tuple[str, ...]:
        """Resolve the current execution path directly from the live machine."""
        return self._resolve_current_execution_path(tuple())

    def get_last_transition(
        self,
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        """Return the most recently observed transition."""
        return self._last_transition

    def get_logs(self) -> list[str]:
        """Return a copy of the collected runtime log lines."""
        with self._log_buffer_lock:
            return list(self._log_entries)

    def get_log_level_name(self) -> str:
        """Return the active YASMIN log level as an uppercase string."""
        return str(yasmin.log_level_to_name(self._log_level)).upper()

    def set_log_level(
        self,
        level: yasmin.LogLevel | str,
        emit_status: bool = True,
    ) -> None:
        """Update the runtime log level and re-apply the logger callback.

        Args:
            level: Target log level as a ``yasmin.LogLevel`` or its name.
            emit_status: Whether the change should be mirrored into the log view.
        """
        if isinstance(level, str):
            normalized_level = LOG_LEVEL_BY_NAME.get(level.strip().upper())
            if normalized_level is None:
                raise ValueError(f"Unsupported log level: {level}")
            level = normalized_level

        self._log_level = level
        self._configure_logging()
        if emit_status:
            self._append_log(f"[STATUS] Log level set to {self.get_log_level_name()}")

    def _configure_logging(self) -> None:
        """Install the runtime logger callback and apply the selected level."""
        yasmin.set_loggers(self._handle_yasmin_log)
        yasmin.set_log_level(self._log_level)

    def _handle_yasmin_log(
        self,
        level: Any,
        file: str,
        function: str,
        line: int,
        text: str,
    ) -> None:
        """Forward YASMIN logs to the terminal and the runtime log view."""
        message = (
            f"[{yasmin.log_level_to_name(level)}] " f"[{file}:{function}:{line}] {text}"
        )
        self._append_log(message)

    def create_sm_from_file(self, path: str) -> bool:
        """Load a runtime state machine from an XML file."""
        self.shutdown(reset_disposed=False)
        self._disposed = False
        self._clear_logs()

        try:
            self._configure_logging()
            self.sm = self.factory.create_sm_from_file(path)
            self._register_callbacks()
        except Exception as exc:
            self.sm = None
            self.ready_changed.emit(False)
            self.error_occurred.emit(f"Failed to create runtime state machine:\n{exc}")
            return False

        self._running = False
        self._blocked = False
        self._finished = False
        self._final_outcome = None
        self._pause_requested = False
        self._shutting_down = False
        self._step_mode = False
        self._set_last_transition(None)
        self._set_active_path(self._resolve_initial_active_path())
        self.ready_changed.emit(self.is_ready())
        self.status_changed.emit("Runtime state machine loaded")
        return self.is_ready()

    def play(self) -> None:
        """Start execution or resume a paused runtime."""
        if not self.is_ready() or self.is_finished() or self._disposed:
            return

        self._resume(step_once=False)
        if self._running:
            self.status_changed.emit("Runtime resumed")
            return

        self._start_execution_thread()
        self.status_changed.emit("Runtime started")

    def pause(self) -> None:
        """Request a pause at the next state boundary."""
        if (
            not self.is_ready()
            or not self._running
            or self.is_finished()
            or self._disposed
        ):
            return
        with self._pause_condition:
            self._pause_requested = True
        self.status_changed.emit("Pause requested")

    def play_once(self) -> None:
        """Execute exactly one transition callback and then pause."""
        if not self.is_ready() or self.is_finished() or self._disposed:
            return

        self._resume(step_once=True)
        if self._running:
            self.status_changed.emit("Runtime will execute one state")
            return

        self._start_execution_thread()
        self.status_changed.emit("Runtime started")

    def cancel_state(self) -> None:
        """Request cancellation of the currently active state."""
        if not self.is_ready() or self.is_finished() or self._disposed:
            return
        try:
            self.sm.cancel_state()
            self.status_changed.emit("Canceling current state")
        except Exception as exc:
            self.error_occurred.emit(f"Failed to cancel current state:\n{exc}")

    def cancel(self) -> None:
        """Continuously request cancellation until the machine stops."""
        if not self.is_ready() or self.is_finished() or self._disposed:
            return

        self._resume(step_once=False)

        def _cancel_worker() -> None:
            while (
                self.is_ready()
                and self._running
                and not self._shutting_down
                and not self._disposed
            ):
                try:
                    self.sm.cancel_state()
                except Exception as exc:
                    self.error_occurred.emit(
                        f"Failed to cancel runtime state machine:\n{exc}"
                    )
                    return
                time.sleep(0.05)

        threading.Thread(
            target=_cancel_worker,
            name="yasmin-runtime-cancel",
            daemon=True,
        ).start()
        self.status_changed.emit("Canceling runtime state machine")

    def kill(self) -> None:
        """Stop the runtime immediately and clear the loaded machine."""
        if (not self.is_ready() and self._execution_thread is None) or self._disposed:
            return

        self._shutting_down = True
        self._resume(step_once=False)

        sm = self.sm
        self.sm = None
        self.bb = None
        self._finished = False
        self._final_outcome = None
        self._step_mode = False

        if sm is not None:
            try:
                sm.cancel_state()
            except Exception:
                pass

        self._set_running(False)
        self._set_blocked(False)
        self._set_active_path(tuple())
        self._set_last_transition(None)
        self.ready_changed.emit(False)
        self.status_changed.emit("Runtime stopped")

    def shutdown(self, reset_disposed: bool = True) -> None:
        """Tear down the runtime and optionally mark it as disposed."""
        if self._disposed and reset_disposed:
            return

        self.kill()
        if self._execution_thread is not None and self._execution_thread.is_alive():
            self._execution_thread.join(timeout=1.0)
        self._execution_thread = None
        self._pause_requested = False
        self._shutting_down = False
        self._step_mode = False

        if reset_disposed:
            self._disposed = True

    def _start_execution_thread(self) -> None:
        """Create the worker thread when execution starts for the first time."""
        if self._execution_thread is not None and self._execution_thread.is_alive():
            return

        initial_path = self._active_path or self._resolve_initial_active_path()
        if initial_path:
            self._set_running(True)
            self._set_active_path(initial_path)

        self._shutting_down = False
        self._execution_thread = threading.Thread(
            target=self._execute_worker,
            name="yasmin-runtime",
            daemon=True,
        )
        self._execution_thread.start()

    def _resume(self, step_once: bool) -> None:
        """Resume execution and configure optional single-step behavior."""
        with self._pause_condition:
            self._step_mode = step_once
            self._pause_requested = False
            self._pause_condition.notify_all()
        if self._blocked:
            self._set_blocked(False)

    def _set_running(self, value: bool) -> None:
        """Update the running flag and emit the corresponding signal."""
        if self._running == value:
            return
        self._running = value
        self.running_changed.emit(value)

    def _set_blocked(self, value: bool) -> None:
        """Update the paused flag and emit the corresponding signal."""
        if self._blocked == value:
            return
        self._blocked = value
        self.blocked_changed.emit(value)

    def _set_active_path(self, state_path: Iterable[str]) -> None:
        """Store the active state path and notify the UI if it changed."""
        next_path = tuple(state_path)
        if self._active_path == next_path:
            return
        self._active_path = next_path
        self.active_state_changed.emit(next_path)

    def _set_last_transition(
        self,
        transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    ) -> None:
        """Store the most recent transition and append a readable log line."""
        if self._last_transition == transition:
            return
        self._last_transition = transition
        self.active_transition_changed.emit(transition)

    def _clear_logs(self) -> None:
        """Reset the in-memory runtime log buffer."""
        with self._log_buffer_lock:
            self._log_entries.clear()
            self._last_log_message = ""
            self._last_log_timestamp = 0.0
        self.log_cleared.emit()

    def _append_log(self, message: str) -> None:
        """Append a log line while filtering accidental duplicate bursts."""
        if self._disposed:
            return

        clean_message = str(message).rstrip()
        if not clean_message:
            return

        with self._log_buffer_lock:
            now = time.monotonic()
            if (
                clean_message == self._last_log_message
                and now - self._last_log_timestamp < 0.02
            ):
                return

            self._last_log_message = clean_message
            self._last_log_timestamp = now
            self._log_entries.append(clean_message)

        self.log_appended.emit(clean_message)

    def _resolve_container(self, path: tuple[str, ...]) -> Optional[Any]:
        """Resolve a container object for the given path inside the live machine."""
        return resolve_container(self.sm, path)

    def _expand_to_deepest_known_path(
        self, base_path: tuple[str, ...]
    ) -> tuple[str, ...]:
        """Expand a container path down to the deepest active child path."""
        return expand_to_deepest_known_path(self.sm, base_path)

    def _resolve_initial_active_path(self) -> tuple[str, ...]:
        """Return the initial path that should be highlighted before execution."""
        return self._expand_to_deepest_known_path(tuple())

    def _resolve_current_execution_path(
        self, fallback: tuple[str, ...]
    ) -> tuple[str, ...]:
        """Resolve the best available live execution path for the UI."""
        live_path = self._expand_to_deepest_known_path(tuple())
        return live_path if live_path else fallback

    def _pause_if_requested(self) -> None:
        """Block the worker thread while a pause request is active."""
        with self._pause_condition:
            if not self._pause_requested or self._shutting_down:
                return

            self._set_blocked(True)
            self.status_changed.emit("Runtime paused")

            while self._pause_requested and not self._shutting_down:
                self._pause_condition.wait()

        self._set_blocked(False)

    def _execute_worker(self) -> None:
        """Run the loaded state machine inside the worker thread."""
        sm = self.sm
        if sm is None or self._disposed:
            return

        try:
            self._configure_logging()
            sm()
        except Exception as exc:
            if not self._disposed:
                self._set_running(False)
                self._set_blocked(False)
                self.error_occurred.emit(f"Runtime execution failed:\n{exc}")
        finally:
            with self._worker_state_lock:
                if threading.current_thread() is self._execution_thread:
                    self._execution_thread = None
                if not self._running and not self._disposed:
                    self.ready_changed.emit(self.is_ready())

    def _register_callbacks(self) -> None:
        """Attach runtime callbacks to all containers in the loaded machine."""
        if self.sm is None:
            return

        visited: set[int] = set()

        def walk(container: Any, prefix: tuple[str, ...]) -> None:
            if container is None:
                return

            object_id = id(container)
            if object_id in visited:
                return
            visited.add(object_id)

            add_start_cb = getattr(container, "add_start_cb", None)
            add_transition_cb = getattr(container, "add_transition_cb", None)
            add_end_cb = getattr(container, "add_end_cb", None)

            if callable(add_start_cb):
                add_start_cb(
                    lambda bb, start_state, current_prefix=prefix: self.start_cb(
                        bb,
                        start_state,
                        current_prefix,
                    )
                )
            if callable(add_transition_cb):
                add_transition_cb(
                    lambda bb, from_state, to_state, outcome, current_prefix=prefix: self.transition_cb(
                        bb,
                        from_state,
                        to_state,
                        outcome,
                        current_prefix,
                    )
                )
            if callable(add_end_cb):
                add_end_cb(
                    lambda bb, outcome, current_prefix=prefix: self.end_cb(
                        bb,
                        outcome,
                        current_prefix,
                    )
                )

            for state_name, nested_state in container_states(container).items():
                if is_container_object(nested_state):
                    walk(nested_state, prefix + (str(state_name),))

        walk(self.sm, tuple())

    def start_cb(
        self,
        bb: Blackboard,
        start_state: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        """Handle a state start callback from the live runtime."""
        if self._disposed:
            return
        self._append_log(
            f"[START] {' / '.join(prefix)} with start_state: {start_state}"
        )

        self._set_running(True)
        active_path = self._expand_to_deepest_known_path(prefix + (str(start_state),))
        self._set_active_path(active_path)

    def end_cb(
        self,
        bb: Blackboard,
        outcome: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        """Handle the completion of a container or the root machine."""
        if self._disposed:
            return
        self._append_log(f"[END] {' / '.join(prefix)} with {outcome}")

        if prefix:
            self._set_active_path(prefix)
            self._pause_if_requested()
            return

        with self._pause_condition:
            self._pause_requested = False
            self._step_mode = False
            self._pause_condition.notify_all()

        self._finished = True
        self._final_outcome = str(outcome)
        self._set_running(False)
        self._set_blocked(False)
        self._set_last_transition(None)
        self.outcome_changed.emit(self._final_outcome)
        self.status_changed.emit(f"Runtime finished with outcome: {outcome}")

    def transition_cb(
        self,
        bb: Blackboard,
        from_state: str,
        to_state: str,
        outcome: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        """Handle transitions and expose the current blackboard safely."""
        if self._disposed:
            return

        # Store blackboard remappings temporarily so the editor can inspect the
        # raw key space without mutating the runtime-visible mapping.
        remappings = dict(bb.get_remappings())

        try:
            bb.set_remappings({})
            self.bb = bb

            from_path = prefix + (str(from_state),)
            to_path = prefix + (str(to_state),)

            self._append_log(
                f"[TRANSITION] {' / '.join(from_path)} --[{outcome}]--> {' / '.join(to_path)}"
            )
            self._set_last_transition((from_path, to_path, str(outcome)))

            with self._pause_condition:
                if self._step_mode:
                    self._step_mode = False
                    self._pause_requested = True

            self._set_active_path(self._expand_to_deepest_known_path(to_path))
            self._pause_if_requested()
        finally:
            try:
                bb.set_remappings(remappings)
            finally:
                self.bb = None
