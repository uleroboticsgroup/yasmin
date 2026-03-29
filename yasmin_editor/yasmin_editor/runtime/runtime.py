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

import importlib
import logging
import threading
import time
from typing import Any, Callable, Iterable, Optional

from PyQt5.QtCore import QObject, pyqtSignal

from yasmin import Blackboard, StateMachine
from yasmin_factory import YasminFactory


class _RuntimeLogHandler(logging.Handler):
    def __init__(self, runtime: "Runtime") -> None:
        super().__init__()
        self.runtime = runtime
        self.setFormatter(logging.Formatter("%(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record)
        except Exception:
            message = record.getMessage()
        self.runtime._append_log(message)


class Runtime(QObject):
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
        super().__init__()
        self.factory = YasminFactory()
        self.sm: Optional[StateMachine] = None
        self.bb: Optional[Blackboard] = None
        self._running = False
        self._blocked = False
        self._finished = False
        self._final_outcome: Optional[str] = None
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

        self._log_entries: list[str] = []
        self._last_log_message = ""
        self._last_log_timestamp = 0.0

        self._yasmin_module: Any = None
        self._yasmin_logs_module: Any = None
        self._python_log_handler: Optional[_RuntimeLogHandler] = None

        self._install_yasmin_loggers()

    def is_ready(self) -> bool:
        return isinstance(self.sm, StateMachine)

    def is_running(self) -> bool:
        return self._running

    def is_blocked(self) -> bool:
        return self._blocked

    def is_finished(self) -> bool:
        return self._finished

    def get_final_outcome(self) -> Optional[str]:
        return self._final_outcome

    def get_status_label(self) -> str:
        if self._finished and self._final_outcome:
            return self._final_outcome
        if self.is_running() and self.is_blocked():
            return "Paused"
        if self.is_running():
            return "Running"
        if self.is_ready():
            return "Ready"
        return "Inactive"

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

    def get_logs(self) -> list[str]:
        return list(self._log_entries)

    def create_sm_from_file(self, path: str) -> bool:
        self.shutdown()
        self._clear_logs()
        self._install_yasmin_loggers()

        try:
            self.sm = self.factory.create_sm_from_file(path)
            self._install_yasmin_loggers()
            self._register_callbacks()
        except Exception as exc:
            self.sm = None
            self.ready_changed.emit(False)
            self.error_occurred.emit(f"Failed to create runtime state machine:\n{exc}")
            return False

        self._finished = False
        self._final_outcome = None
        self._set_last_transition(None)
        self._set_active_path(self._resolve_initial_active_path())
        self.ready_changed.emit(self.is_ready())
        self.status_changed.emit("Runtime state machine loaded")
        return self.is_ready()

    def play(self) -> None:
        if not self.is_ready() or self.is_finished():
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
        if not self.is_ready() or not self._running or self.is_finished():
            return
        with self._pause_condition:
            self._pause_requested = True
        self.status_changed.emit("Pause requested")

    def play_once(self) -> None:
        if not self.is_ready() or self.is_finished():
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
        if not self.is_ready() or self.is_finished():
            return
        try:
            self.sm.cancel_state()
            self.status_changed.emit("Canceling current state")
        except Exception as exc:
            self.error_occurred.emit(f"Failed to cancel current state:\n{exc}")

    def cancel(self) -> None:
        if not self.is_ready() or self.is_finished():
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
        self._finished = False
        self._final_outcome = None

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

    def _clear_logs(self) -> None:
        self._log_entries.clear()
        self._last_log_message = ""
        self._last_log_timestamp = 0.0
        self.log_cleared.emit()

    def _append_log(self, message: str) -> None:
        clean_message = str(message).rstrip()
        if not clean_message:
            return

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

    def _install_python_logging_bridge(self) -> None:
        root_logger = logging.getLogger()

        if self._python_log_handler is None:
            self._python_log_handler = _RuntimeLogHandler(self)
            self._python_log_handler.setLevel(logging.NOTSET)

        if self._python_log_handler not in root_logger.handlers:
            root_logger.addHandler(self._python_log_handler)

        if root_logger.level > logging.NOTSET:
            root_logger.setLevel(logging.NOTSET)

    def _install_yasmin_loggers(self) -> None:
        self._install_python_logging_bridge()

        try:
            self._yasmin_module = importlib.import_module("yasmin")
        except Exception:
            self._yasmin_module = None

        try:
            self._yasmin_logs_module = importlib.import_module("yasmin.logs")
        except Exception:
            self._yasmin_logs_module = None

        self._patch_yasmin_log_functions()
        self._patch_yasmin_setters()
        self._activate_yasmin_logger()

    def _patch_yasmin_log_functions(self) -> None:
        for func_name, level_name in (
            ("log_error", "ERROR"),
            ("log_warn", "WARN"),
            ("log_info", "INFO"),
            ("log_debug", "DEBUG"),
        ):
            original = None

            if self._yasmin_logs_module is not None:
                original = getattr(self._yasmin_logs_module, func_name, None)

            if original is None and self._yasmin_module is not None:
                original = getattr(self._yasmin_module, func_name, None)

            if not callable(original):
                continue

            if getattr(original, "__yasmin_editor_runtime_wrapper__", False):
                continue

            wrapped = self._build_log_function_wrapper(
                func_name=func_name,
                level_name=level_name,
                original=original,
            )

            if self._yasmin_logs_module is not None:
                setattr(self._yasmin_logs_module, func_name, wrapped)

            if self._yasmin_module is not None:
                setattr(self._yasmin_module, func_name, wrapped)

    def _build_log_function_wrapper(
        self,
        func_name: str,
        level_name: str,
        original: Callable[..., Any],
    ) -> Callable[..., Any]:
        def wrapped(file: str, function: str, line: int, text: str) -> Any:
            self._append_log(f"[{level_name}] [{file}:{function}:{line}] {text}")
            return original(file, function, line, text)

        wrapped.__name__ = func_name
        wrapped.__yasmin_editor_runtime_wrapper__ = True
        return wrapped

    def _patch_yasmin_setters(self) -> None:
        original_set_loggers = None

        if self._yasmin_logs_module is not None:
            original_set_loggers = getattr(
                self._yasmin_logs_module, "set_loggers", None
            )

        if original_set_loggers is None and self._yasmin_module is not None:
            original_set_loggers = getattr(self._yasmin_module, "set_loggers", None)

        if callable(original_set_loggers) and not getattr(
            original_set_loggers, "__yasmin_editor_runtime_wrapper__", False
        ):

            def wrapped_set_loggers(user_logger: Callable[..., Any]) -> Any:
                if user_logger is self._yasmin_log_message:
                    return original_set_loggers(self._yasmin_log_message)

                def combined_logger(
                    level: Any,
                    file: str,
                    function: str,
                    line: int,
                    text: str,
                ) -> None:
                    self._yasmin_log_message(level, file, function, line, text)
                    user_logger(level, file, function, line, text)

                return original_set_loggers(combined_logger)

            wrapped_set_loggers.__yasmin_editor_runtime_wrapper__ = True

            if self._yasmin_logs_module is not None:
                setattr(self._yasmin_logs_module, "set_loggers", wrapped_set_loggers)

            if self._yasmin_module is not None:
                setattr(self._yasmin_module, "set_loggers", wrapped_set_loggers)

        self._wrap_reset_logger_function("set_default_loggers")
        self._wrap_reset_logger_function("set_py_loggers")

    def _wrap_reset_logger_function(self, func_name: str) -> None:
        original = None

        if self._yasmin_logs_module is not None:
            original = getattr(self._yasmin_logs_module, func_name, None)

        if original is None and self._yasmin_module is not None:
            original = getattr(self._yasmin_module, func_name, None)

        if not callable(original):
            return

        if getattr(original, "__yasmin_editor_runtime_wrapper__", False):
            return

        def wrapped(*args: Any, **kwargs: Any) -> Any:
            result = original(*args, **kwargs)
            self._activate_yasmin_logger()
            return result

        wrapped.__name__ = func_name
        wrapped.__yasmin_editor_runtime_wrapper__ = True

        if self._yasmin_logs_module is not None and hasattr(
            self._yasmin_logs_module, func_name
        ):
            setattr(self._yasmin_logs_module, func_name, wrapped)

        if self._yasmin_module is not None and hasattr(self._yasmin_module, func_name):
            setattr(self._yasmin_module, func_name, wrapped)

    def _activate_yasmin_logger(self) -> None:
        set_loggers = None

        if self._yasmin_logs_module is not None:
            set_loggers = getattr(self._yasmin_logs_module, "set_loggers", None)

        if not callable(set_loggers) and self._yasmin_module is not None:
            set_loggers = getattr(self._yasmin_module, "set_loggers", None)

        if callable(set_loggers):
            try:
                set_loggers(self._yasmin_log_message)
            except Exception:
                pass

    def _yasmin_log_message(
        self,
        level: Any,
        file: str,
        function: str,
        line: int,
        text: str,
    ) -> None:
        level_name = self._log_level_to_name(level)
        self._append_log(f"[{level_name}] [{file}:{function}:{line}] {text}")

    def _log_level_to_name(self, level: Any) -> str:
        if self._yasmin_logs_module is not None:
            log_level_to_name = getattr(
                self._yasmin_logs_module, "log_level_to_name", None
            )
            if callable(log_level_to_name):
                try:
                    return str(log_level_to_name(level))
                except Exception:
                    pass

        if hasattr(level, "name"):
            return str(level.name)

        if isinstance(level, str):
            return level.upper()

        try:
            value = int(level)
        except Exception:
            value = None

        if value is not None:
            mapping = {0: "ERROR", 1: "WARN", 2: "INFO", 3: "DEBUG"}
            return mapping.get(value, str(value))

        return str(level)

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
            self._install_yasmin_loggers()
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

    def _resolve_container(self, prefix: tuple[str, ...]) -> Optional[Any]:
        container: Any = self.sm
        for state_name in prefix:
            if container is None:
                return None
            get_states = getattr(container, "get_states", None)
            if not callable(get_states):
                return None
            try:
                states = get_states()
            except Exception:
                return None
            if not hasattr(states, "get"):
                return None
            container = states.get(state_name)
        return container

    def _container_has_state(self, prefix: tuple[str, ...], state_name: str) -> bool:
        container = self._resolve_container(prefix)
        if container is None:
            return False

        get_states = getattr(container, "get_states", None)
        if callable(get_states):
            try:
                states = get_states()
            except Exception:
                states = None
            if states is not None and hasattr(states, "__contains__"):
                try:
                    return state_name in states
                except Exception:
                    return False
        return False

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
            return

        with self._pause_condition:
            self._pause_requested = False
            self._step_once_requested = False
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
        self.bb = bb
        from_name = str(from_state)
        to_name = str(to_state)

        self._set_last_transition(
            (
                prefix + (from_name,),
                prefix + (to_name,),
                str(outcome),
            )
        )

        if self._container_has_state(prefix, to_name):
            self._set_active_path(prefix + (to_name,))
        else:
            self._set_active_path(prefix + (from_name,))

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
