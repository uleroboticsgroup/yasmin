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
from typing import Any, Iterable, Optional

from PyQt5.QtCore import QObject, pyqtSignal
from yasmin_editor.runtime.logging import RuntimeLogger
from yasmin_editor.runtime.traversal import (
    child_state,
    container_states,
    expand_to_deepest_known_path,
    is_concurrence_object,
    is_container_object,
    resolve_container,
)

import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_factory import YasminFactory


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
        self.bb: Blackboard = Blackboard()

        self._running = False
        self._blocked = False
        self._finished = False
        self._final_outcome: Optional[str] = None
        self._disposed = False
        self._current_state_ref: Optional[Any] = None
        self._last_state_ref: Optional[Any] = None
        self._shutting_down = False

        self._active_path: tuple[str, ...] = tuple()
        self._last_transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]] = (
            None
        )

        self._execution_thread: Optional[threading.Thread] = None
        self._worker_state_lock = threading.Lock()
        self._pause_condition = threading.Condition()
        self._pause_requested = False

        self._step_mode = False
        self._breakpoint_lock = threading.Lock()
        self._breakpoints_before: set[tuple[str, ...]] = set()
        self._pause_status_message: Optional[str] = None

        self.logger = RuntimeLogger(
            append_callback=self.log_appended.emit,
            clear_callback=self.log_cleared.emit,
            is_disposed=lambda: self._disposed,
        )

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
        if self._finished:
            return None
        return self._active_path[-1] if self._active_path else None

    def get_current_state_ref(self) -> Optional[Any]:
        """Return the currently highlighted runtime state object."""
        return self._current_state_ref

    def get_last_state_ref(self) -> Optional[Any]:
        """Return the previously active runtime state object."""
        return self._last_state_ref

    def get_active_path(self) -> tuple[str, ...]:
        """Return the last known active path inside the loaded machine."""
        return self._active_path

    def get_last_transition(
        self,
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        """Return the most recently observed transition."""
        return self._last_transition

    def get_logs(self) -> list[str]:
        """Return a copy of the collected runtime log lines."""
        return self.logger.get_logs()

    def get_log_level_name(self) -> str:
        """Return the active YASMIN log level as an uppercase string."""
        return self.logger.get_log_level_name()

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
        self.logger.set_log_level(level, emit_status=emit_status)

    def create_sm_from_file(self, path: str) -> bool:
        """Load a runtime state machine from an XML file."""
        self.shutdown(reset_disposed=False)
        self._disposed = False
        self.logger.clear()

        try:
            self.logger.configure()
            self.sm = self.factory.create_sm_from_file(path)
            self.bb = Blackboard()
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
        self.logger.reset_depth()
        self._set_last_transition(None)
        initial_active_path = self._resolve_initial_active_path()
        self._set_active_path(initial_active_path)
        self._last_state_ref = None
        self._current_state_ref = self._resolve_state_reference(initial_active_path)
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

    def set_breakpoints(
        self,
        before_paths: Iterable[Iterable[str]] = tuple(),
    ) -> None:
        """Replace the active breakpoint set used by the runtime worker."""
        with self._breakpoint_lock:
            self._breakpoints_before = {
                tuple(str(item) for item in path) for path in before_paths
            }

    def cancel_state(self) -> None:
        """Request cancellation of the currently active state."""
        if not self.is_ready() or self.is_finished() or self._disposed:
            return
        try:
            self.sm.cancel_state()
            self.status_changed.emit("Canceling current state")
        except Exception as exc:
            self.error_occurred.emit(f"Failed to cancel current state:\n{exc}")

    def cancel_state_machine(self) -> None:
        """Request cancellation of the complete runtime state machine."""
        if (
            not self.is_ready()
            or not self.is_running()
            or self.is_finished()
            or self._disposed
        ):
            return

        try:
            self.sm.cancel_state_machine()
            self.status_changed.emit("Canceling runtime state machine")
        except Exception as exc:
            self.error_occurred.emit(f"Failed to cancel runtime state machine:\n{exc}")

    def shutdown(self, reset_disposed: bool = True) -> None:
        """Tear down the runtime and optionally mark it as disposed."""
        if self._disposed and reset_disposed:
            return

        self._shutting_down = True

        with self._pause_condition:
            self._pause_requested = False
            self._step_mode = False
            self._pause_condition.notify_all()

        if self.sm is not None:
            try:
                self.sm.cancel_state_machine()
            except Exception:
                try:
                    self.sm.cancel_state()
                except Exception:
                    pass

        if self._execution_thread is not None and self._execution_thread.is_alive():
            self._execution_thread.join(timeout=1.0)

        self.sm = None
        self._execution_thread = None
        self._finished = False
        self._final_outcome = None
        self._pause_requested = False
        self._shutting_down = False
        self._step_mode = False
        self.logger.reset_depth()

        self._set_running(False)
        self._set_blocked(False)
        self._set_active_path(tuple())
        self._set_last_transition(None)
        self._current_state_ref = None
        self._last_state_ref = None
        self.ready_changed.emit(False)

        if reset_disposed:
            self._disposed = True

    def _resolve_state_machine_cancel_exception_types(
        self,
    ) -> tuple[type[BaseException], ...]:
        """Return known Python exception types for full state-machine cancelation."""
        exception_types: list[type[BaseException]] = []

        for owner in (yasmin, getattr(yasmin, "state_machine", None)):
            exception_type = getattr(owner, "StateMachineCancelException", None)
            if isinstance(exception_type, type) and issubclass(
                exception_type, BaseException
            ):
                exception_types.append(exception_type)

        return tuple(dict.fromkeys(exception_types))

    def _is_state_machine_cancel_exception(self, exc: BaseException) -> bool:
        """Return whether *exc* represents the dedicated hard-cancel condition."""
        known_types = self._resolve_state_machine_cancel_exception_types()
        if known_types and isinstance(exc, known_types):
            return True

        if exc.__class__.__name__ == "StateMachineCancelException":
            return True

        return str(exc).startswith("State machine canceled:")

    def _finalize_runtime_completion(
        self,
        final_outcome: str,
        status_message: str,
    ) -> None:
        """Mark the runtime as finished and publish the terminal status."""
        with self._pause_condition:
            self._pause_requested = False
            self._step_mode = False
            self._pause_condition.notify_all()

        final_active_path = tuple(self._active_path)
        if final_active_path:
            self._set_active_path(final_active_path)
            self._last_state_ref = self._resolve_state_reference(final_active_path)

        self._finished = True
        self._final_outcome = str(final_outcome)
        self._set_running(False)
        self._set_blocked(False)
        self._set_last_transition(None)
        self._current_state_ref = None
        self.outcome_changed.emit(self._final_outcome)
        self.status_changed.emit(status_message)

    def _handle_state_machine_canceled(self, exc: BaseException) -> None:
        """Treat a hard state-machine cancel as an expected terminal condition."""
        if self._disposed:
            return

        self.logger.append(f"[CANCEL] {exc}", is_end=True)
        self._finalize_runtime_completion(
            final_outcome="Canceled",
            status_message="Runtime canceled",
        )

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
            self._pause_status_message = None
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

    def _resolve_state_reference(self, path: tuple[str, ...]) -> Optional[Any]:
        """Resolve a live state object from a runtime path."""
        if self.sm is None or not path:
            return None

        container = resolve_container(self.sm, path[:-1])
        if container is None:
            return None

        return child_state(container, path[-1])

    def _update_shell_state_refs(
        self,
        current_path: tuple[str, ...],
        last_path: Optional[tuple[str, ...]] = None,
    ) -> None:
        """Update the live state references exposed in the shell."""
        if last_path is not None:
            self._last_state_ref = self._resolve_state_reference(last_path)
        self._current_state_ref = self._resolve_state_reference(current_path)

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

    def _pause_if_requested(self) -> None:
        """Block the worker thread while a pause request is active."""
        with self._pause_condition:
            if not self._pause_requested or self._shutting_down:
                return

            self._set_blocked(True)
            pause_message = self._pause_status_message or "Runtime paused"
            self.status_changed.emit(pause_message)

            while self._pause_requested and not self._shutting_down:
                self._pause_condition.wait()

            self._pause_status_message = None

        self._set_blocked(False)

    def _request_pause(self, status_message: Optional[str] = None) -> None:
        """Request a runtime pause and optionally override the pause status text."""
        with self._pause_condition:
            self._pause_requested = True
            if status_message:
                self._pause_status_message = str(status_message)

    def _has_breakpoint(self, state_path: tuple[str, ...]) -> bool:
        """Return whether a breakpoint exists for the given state path."""
        normalized = tuple(str(item) for item in state_path)
        with self._breakpoint_lock:
            return normalized in self._breakpoints_before

    def _request_breakpoint_pause(self, state_path: tuple[str, ...]) -> bool:
        """Arm a pause request when a matching breakpoint is reached."""
        normalized = tuple(str(item) for item in state_path)
        if not normalized or not self._has_breakpoint(normalized):
            return False

        state_label = " / ".join(normalized)
        self._request_pause(f"Paused at breakpoint: {state_label}")
        return True

    def _execute_worker(self) -> None:
        """Run the loaded state machine inside the worker thread."""
        sm = self.sm
        if sm is None or self._disposed:
            return

        try:
            self.logger.configure()
            sm(self.bb)
        except Exception as exc:
            if self._is_state_machine_cancel_exception(exc):
                self._handle_state_machine_canceled(exc)
            elif not self._disposed:
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

            if is_concurrence_object(container):
                return

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

        self.logger.append(
            f"[START] {' / '.join(prefix) if prefix else '<root>'} "
            f"with start_state: {start_state}"
        )
        self.logger.increment_depth()

        self._set_running(True)
        active_path = self._expand_to_deepest_known_path(prefix + (str(start_state),))
        self._set_active_path(active_path)
        self._current_state_ref = self._resolve_state_reference(active_path)
        self._request_breakpoint_pause(active_path)
        self._pause_if_requested()

    def end_cb(
        self,
        bb: Blackboard,
        outcome: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        """Handle the completion of a container or the root machine."""
        if self._disposed:
            return

        self.logger.append(
            f"[END] {' / '.join(prefix) if prefix else '<root>'} with outcome: {outcome}",
            is_end=True,
        )
        self.logger.decrement_depth()

        if prefix:
            self._set_active_path(prefix)
            self._current_state_ref = self._resolve_state_reference(prefix)
            self._pause_if_requested()
            return

        self._finalize_runtime_completion(
            final_outcome=str(outcome),
            status_message=f"Runtime finished with outcome: {outcome}",
        )

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

            from_path = prefix + (str(from_state),)
            to_path = prefix + (str(to_state),)

            self.logger.append(
                f"[TRANSITION] {' / '.join(from_path)} "
                f"--[{outcome}]--> {' / '.join(to_path)}"
            )
            self._set_last_transition((from_path, to_path, str(outcome)))

            self._set_active_path(from_path)
            self._current_state_ref = self._resolve_state_reference(from_path)

            with self._pause_condition:
                if self._step_mode:
                    self._step_mode = False
                    self._pause_requested = True

            if self._has_breakpoint(to_path):
                self._update_shell_state_refs(to_path, from_path)
                self._set_active_path(to_path)
                self._current_state_ref = self._resolve_state_reference(to_path)
                self._request_breakpoint_pause(to_path)
                self._pause_if_requested()

            active_path = self._expand_to_deepest_known_path(to_path)
            self._update_shell_state_refs(active_path, from_path)
            self._set_active_path(active_path)
            self._pause_if_requested()
        finally:
            bb.set_remappings(remappings)
