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
import weakref
from typing import Any, Callable, Optional


class _RuntimeLogHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self.setFormatter(logging.Formatter("%(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record)
        except Exception:
            message = record.getMessage()
        RuntimeLoggingBridge.append_log_to_current_runtime(message)


class RuntimeLoggingBridge:
    _current_runtime_ref: Optional[weakref.ReferenceType] = None
    _python_log_handler: Optional[_RuntimeLogHandler] = None
    _yasmin_module: Any = None
    _yasmin_logs_module: Any = None
    _global_patch_installed = False
    _global_patch_lock = threading.Lock()

    @classmethod
    def set_current_runtime(cls, runtime: Optional[Any]) -> None:
        cls._current_runtime_ref = None if runtime is None else weakref.ref(runtime)

    @classmethod
    def get_current_runtime(cls) -> Optional[Any]:
        if cls._current_runtime_ref is None:
            return None
        runtime = cls._current_runtime_ref()
        if runtime is None:
            cls._current_runtime_ref = None
            return None
        if getattr(runtime, "_disposed", False):
            return None
        return runtime

    @classmethod
    def append_log_to_current_runtime(cls, message: str) -> None:
        runtime = cls.get_current_runtime()
        if runtime is not None:
            runtime._append_log(message)

    @classmethod
    def install(cls) -> None:
        with cls._global_patch_lock:
            if cls._python_log_handler is None:
                cls._python_log_handler = _RuntimeLogHandler()
                cls._python_log_handler.setLevel(logging.NOTSET)

            root_logger = logging.getLogger()
            if cls._python_log_handler not in root_logger.handlers:
                root_logger.addHandler(cls._python_log_handler)
            if root_logger.level > logging.NOTSET:
                root_logger.setLevel(logging.NOTSET)

            try:
                cls._yasmin_module = importlib.import_module("yasmin")
            except Exception:
                cls._yasmin_module = None

            try:
                cls._yasmin_logs_module = importlib.import_module("yasmin.logs")
            except Exception:
                cls._yasmin_logs_module = None

            if not cls._global_patch_installed:
                cls._patch_yasmin_log_functions()
                cls._patch_yasmin_setters()
                cls._global_patch_installed = True

            cls.activate_yasmin_logger()

    @classmethod
    def _patch_yasmin_log_functions(cls) -> None:
        for func_name, level_name in (
            ("log_error", "ERROR"),
            ("log_warn", "WARN"),
            ("log_info", "INFO"),
            ("log_debug", "DEBUG"),
        ):
            original = None
            if cls._yasmin_logs_module is not None:
                original = getattr(cls._yasmin_logs_module, func_name, None)
            if original is None and cls._yasmin_module is not None:
                original = getattr(cls._yasmin_module, func_name, None)

            if not callable(original) or getattr(
                original,
                "__yasmin_editor_runtime_wrapper__",
                False,
            ):
                continue

            def wrapped(
                file: str,
                function: str,
                line: int,
                text: str,
                *,
                _original: Callable[..., Any] = original,
                _level_name: str = level_name,
            ) -> Any:
                cls.append_log_to_current_runtime(
                    f"[{_level_name}] [{file}:{function}:{line}] {text}"
                )
                return _original(file, function, line, text)

            wrapped.__name__ = func_name
            wrapped.__yasmin_editor_runtime_wrapper__ = True

            if cls._yasmin_logs_module is not None:
                setattr(cls._yasmin_logs_module, func_name, wrapped)
            if cls._yasmin_module is not None:
                setattr(cls._yasmin_module, func_name, wrapped)

    @classmethod
    def _patch_yasmin_setters(cls) -> None:
        original_set_loggers = None
        if cls._yasmin_logs_module is not None:
            original_set_loggers = getattr(cls._yasmin_logs_module, "set_loggers", None)
        if original_set_loggers is None and cls._yasmin_module is not None:
            original_set_loggers = getattr(cls._yasmin_module, "set_loggers", None)

        if callable(original_set_loggers) and not getattr(
            original_set_loggers,
            "__yasmin_editor_runtime_wrapper__",
            False,
        ):

            def wrapped_set_loggers(user_logger: Callable[..., Any]) -> Any:
                if user_logger is cls.yasmin_log_message:
                    return original_set_loggers(cls.yasmin_log_message)

                def combined_logger(
                    level: Any,
                    file: str,
                    function: str,
                    line: int,
                    text: str,
                ) -> None:
                    cls.yasmin_log_message(level, file, function, line, text)
                    user_logger(level, file, function, line, text)

                return original_set_loggers(combined_logger)

            wrapped_set_loggers.__yasmin_editor_runtime_wrapper__ = True

            if cls._yasmin_logs_module is not None:
                setattr(cls._yasmin_logs_module, "set_loggers", wrapped_set_loggers)
            if cls._yasmin_module is not None:
                setattr(cls._yasmin_module, "set_loggers", wrapped_set_loggers)

        cls._wrap_reset_logger_function("set_default_loggers")
        cls._wrap_reset_logger_function("set_py_loggers")

    @classmethod
    def _wrap_reset_logger_function(cls, func_name: str) -> None:
        original = None
        if cls._yasmin_logs_module is not None:
            original = getattr(cls._yasmin_logs_module, func_name, None)
        if original is None and cls._yasmin_module is not None:
            original = getattr(cls._yasmin_module, func_name, None)

        if not callable(original) or getattr(
            original,
            "__yasmin_editor_runtime_wrapper__",
            False,
        ):
            return

        def wrapped(*args: Any, **kwargs: Any) -> Any:
            result = original(*args, **kwargs)
            cls.activate_yasmin_logger()
            return result

        wrapped.__name__ = func_name
        wrapped.__yasmin_editor_runtime_wrapper__ = True

        if cls._yasmin_logs_module is not None and hasattr(cls._yasmin_logs_module, func_name):
            setattr(cls._yasmin_logs_module, func_name, wrapped)
        if cls._yasmin_module is not None and hasattr(cls._yasmin_module, func_name):
            setattr(cls._yasmin_module, func_name, wrapped)

    @classmethod
    def activate_yasmin_logger(cls) -> None:
        set_loggers = None
        if cls._yasmin_logs_module is not None:
            set_loggers = getattr(cls._yasmin_logs_module, "set_loggers", None)
        if not callable(set_loggers) and cls._yasmin_module is not None:
            set_loggers = getattr(cls._yasmin_module, "set_loggers", None)

        if callable(set_loggers):
            try:
                set_loggers(cls.yasmin_log_message)
            except Exception:
                pass

    @classmethod
    def log_level_to_name(cls, level: Any) -> str:
        if cls._yasmin_logs_module is not None:
            log_level_to_name = getattr(cls._yasmin_logs_module, "log_level_to_name", None)
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
            return {0: "ERROR", 1: "WARN", 2: "INFO", 3: "DEBUG"}.get(value, str(value))

        return str(level)

    @classmethod
    def yasmin_log_message(
        cls,
        level: Any,
        file: str,
        function: str,
        line: int,
        text: str,
    ) -> None:
        cls.append_log_to_current_runtime(
            f"[{cls.log_level_to_name(level)}] [{file}:{function}:{line}] {text}"
        )
