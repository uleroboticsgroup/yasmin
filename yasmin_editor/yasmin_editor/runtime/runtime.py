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

from __future__ import annotations

import threading
import time
from typing import Any, Iterable, Optional

from PyQt5.QtCore import QObject, pyqtSignal

from yasmin import Blackboard, StateMachine
from yasmin_factory import YasminFactory


class Runtime(QObject):
    ready_changed = pyqtSignal(bool)
    running_changed = pyqtSignal(bool)
    blocked_changed = pyqtSignal(bool)
    active_state_changed = pyqtSignal(object)
    active_transition_changed = pyqtSignal(object)
    outcome_changed = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.factory = YasminFactory()
        self.sm: Optional[StateMachine] = None
        self.bb: Optional[Blackboard] = None
        self._running = False
        self._blocked = False
        self._active_path: tuple[str, ...] = tuple()
        self._last_transition: Optional[
            tuple[tuple[str, ...], tuple[str, ...], str]
        ] = None
        self._execution_thread: Optional[threading.Thread] = None
        self._pause_condition = threading.Condition()
        self._pause_requested = False
        self._step_once_requested = False
        self._shutting_down = False
        self._worker_state_lock = threading.Lock()

    def is_ready(self) -> bool:
        return isinstance(self.sm, StateMachine)

    def is_running(self) -> bool:
        return self._running

    def is_blocked(self) -> bool:
        return self._blocked

    def get_current_state(self) -> Optional[str]:
        if self._active_path:
            return self._active_path[-1]
        if not self.is_ready():
            return None
        try:
            return self.sm.get_current_state()
        except Exception:
            return None

    def get_active_path(self) -> tuple[str, ...]:
        return self._active_path

    def get_last_transition(
        self,
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        return self._last_transition

    def create_sm_from_file(self, path: str) -> bool:
        self.shutdown()

        try:
            self.sm = self.factory.create_sm_from_file(path)
            self._register_callbacks()
        except Exception as exc:
            self.sm = None
            self.ready_changed.emit(False)
            self.error_occurred.emit(f"Failed to create runtime state machine:\n{exc}")
            return False

        self._set_last_transition(None)
        self._set_active_path(self._resolve_initial_active_path())
        self.ready_changed.emit(self.is_ready())
        self.status_changed.emit("Runtime state machine loaded")
        return self.is_ready()

    def play(self) -> None:
        if not self.is_ready():
            return

        self._resume(step_once=False)

        if self._running:
            self.status_changed.emit("Runtime resumed")
            return

        if self._execution_thread is not None and self._execution_thread.is_alive():
            return

        initial_active_path = self._resolve_initial_active_path()
        if initial_active_path:
            self._set_running(True)
            self._set_active_path(initial_active_path)

        self._shutting_down = False
        self._execution_thread = threading.Thread(
            target=self._execute_worker,
            name="yasmin-runtime",
            daemon=True,
        )
        self._execution_thread.start()
        self.status_changed.emit("Runtime started")

    def pause(self) -> None:
        if not self.is_ready() or not self._running:
            return
        with self._pause_condition:
            self._pause_requested = True
        self.status_changed.emit("Runtime will pause on the next transition")

    def play_once(self) -> None:
        if not self.is_ready():
            return
        self._resume(step_once=True)
        if not self._running:
            if self._execution_thread is not None and self._execution_thread.is_alive():
                return

            initial_active_path = self._resolve_initial_active_path()
            if initial_active_path:
                self._set_running(True)
                self._set_active_path(initial_active_path)

            self._shutting_down = False
            self._execution_thread = threading.Thread(
                target=self._execute_worker,
                name="yasmin-runtime",
                daemon=True,
            )
            self._execution_thread.start()
            self.status_changed.emit("Runtime started")
            return
        self.status_changed.emit("Runtime will execute one state")

    def cancel_state(self) -> None:
        if not self.is_ready():
            return
        try:
            self.sm.cancel_state()
            self.status_changed.emit("Canceling current state")
        except Exception as exc:
            self.error_occurred.emit(f"Failed to cancel current state:\n{exc}")

    def cancel(self) -> None:
        if not self.is_ready():
            return

        self._resume(step_once=False)

        def _cancel_worker() -> None:
            while self.is_ready() and self._running and not self._shutting_down:
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
        if not self.is_ready() and self._execution_thread is None:
            return

        self._shutting_down = True
        self._resume(step_once=False)

        sm = self.sm
        self.sm = None
        self.bb = None

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
        self.status_changed.emit("Runtime force-stop requested")

    def shutdown(self) -> None:
        self.kill()
        if self._execution_thread is not None and self._execution_thread.is_alive():
            self._execution_thread.join(timeout=0.2)
        self._execution_thread = None
        self._pause_requested = False
        self._step_once_requested = False
        self._shutting_down = False

    def _resume(self, step_once: bool) -> None:
        with self._pause_condition:
            self._step_once_requested = step_once
            self._pause_requested = False
            self._pause_condition.notify_all()
        if self._blocked:
            self._set_blocked(False)

    def _set_running(self, value: bool) -> None:
        if self._running == value:
            return
        self._running = value
        self.running_changed.emit(value)

    def _set_blocked(self, value: bool) -> None:
        if self._blocked == value:
            return
        self._blocked = value
        self.blocked_changed.emit(value)

    def _set_active_path(self, state_path: Iterable[str]) -> None:
        next_path = tuple(state_path)
        if self._active_path == next_path:
            return
        self._active_path = next_path
        self.active_state_changed.emit(next_path)

    def _set_last_transition(
        self,
        transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    ) -> None:
        if self._last_transition == transition:
            return
        self._last_transition = transition
        self.active_transition_changed.emit(transition)

    def _resolve_initial_active_path(self) -> tuple[str, ...]:
        container = self.sm
        path: list[str] = []
        visited: set[int] = set()

        while container is not None and id(container) not in visited:
            visited.add(id(container))

            start_state = None
            get_start_state = getattr(container, "get_start_state", None)
            if callable(get_start_state):
                try:
                    start_state = get_start_state()
                except Exception:
                    start_state = None

            if not start_state:
                start_state = getattr(container, "start_state", None)

            if not start_state:
                break

            state_name = str(start_state)
            path.append(state_name)

            get_states = getattr(container, "get_states", None)
            if not callable(get_states):
                break

            try:
                states = get_states()
            except Exception:
                break

            if not hasattr(states, "get"):
                break

            next_container = states.get(state_name)
            if next_container is None:
                break

            child_get_states = getattr(next_container, "get_states", None)
            if not callable(child_get_states):
                break

            container = next_container

        return tuple(path)

    def _execute_worker(self) -> None:
        sm = self.sm
        if sm is None:
            return

        try:
            sm()
        except Exception as exc:
            self._set_running(False)
            self._set_blocked(False)
            self.error_occurred.emit(f"Runtime execution failed:\n{exc}")
        finally:
            with self._worker_state_lock:
                if threading.current_thread() is self._execution_thread:
                    self._execution_thread = None
                if not self._running:
                    self._set_active_path(tuple())
                    self.ready_changed.emit(self.is_ready())

    def _register_callbacks(self) -> None:
        if self.sm is None:
            return

        visited: set[int] = set()

        def walk(container: Any, prefix: tuple[str, ...]) -> None:
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
                        bb, start_state, current_prefix
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

            get_states = getattr(container, "get_states", None)
            if not callable(get_states):
                return

            try:
                states = get_states()
            except Exception:
                return

            state_items = states.items() if hasattr(states, "items") else []
            for state_name, child_state in state_items:
                walk(child_state, prefix + (str(state_name),))

        walk(self.sm, tuple())

    def start_cb(
        self,
        bb: Blackboard,
        start_state: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        self.bb = bb
        self._set_running(True)
        self._set_active_path(prefix + (str(start_state),))

    def end_cb(
        self,
        bb: Blackboard,
        outcome: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        self.bb = bb

        if prefix:
            self._set_active_path(prefix)
            return

        with self._pause_condition:
            self._pause_requested = False
            self._step_once_requested = False
            self._pause_condition.notify_all()

        self._set_running(False)
        self._set_blocked(False)
        self._set_active_path(tuple())
        self._set_last_transition(None)
        self.outcome_changed.emit(outcome)
        self.status_changed.emit(f"Runtime finished with outcome: {outcome}")

    def transition_cb(
        self,
        bb: Blackboard,
        from_state: str,
        to_state: str,
        outcome: str,
        prefix: tuple[str, ...] = tuple(),
    ) -> None:
        self.bb = bb
        self._set_last_transition(
            (
                prefix + (str(from_state),),
                prefix + (str(to_state),),
                str(outcome),
            )
        )
        self._set_active_path(prefix + (str(to_state),))

        with self._pause_condition:
            if self._step_once_requested:
                self._pause_requested = True
                self._step_once_requested = False

            if not self._pause_requested or self._shutting_down:
                return

            self._set_blocked(True)
            self.status_changed.emit("Runtime paused")

            while self._pause_requested and not self._shutting_down:
                self._pause_condition.wait()

        self._set_blocked(False)
