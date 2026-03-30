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

"""
Runtime logging bridge for the YASMIN editor.

This module forwards two log sources into the active runtime instance:

1. Regular Python logging records from the process root logger.
2. Log messages emitted through the YASMIN logging helpers.

The bridge keeps only a weak reference to the current runtime so that the
logging layer does not extend its lifetime. YASMIN monkeypatching is applied
idempotently, and the Python logging handler can be installed and uninstalled
cleanly during runtime startup and shutdown.
"""

from __future__ import annotations

import functools
import importlib
import logging
import threading
import weakref
from typing import Any, Callable, Optional

# Marker attribute used to avoid wrapping the same YASMIN function multiple times.
_WRAPPER_ATTR = "__yasmin_editor_runtime_wrapper__"


class _RuntimeLogHandler(logging.Handler):
    """
    Logging handler that forwards Python log records to the active runtime.

    The handler formats the incoming record to a plain message and delegates
    the final routing decision to ``RuntimeLoggingBridge``.
    """

    def __init__(self) -> None:
        """
        Initialize the handler with a minimal message-only formatter.

        The runtime already formats and stores messages on its own, therefore
        the default formatter only extracts the final record text.
        """
        super().__init__(level=logging.NOTSET)
        self.setFormatter(logging.Formatter("%(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        """
        Forward a Python log record to the active runtime instance.

        Args:
            record: Python logging record produced by the logging framework.
        """
        try:
            message = self.format(record)
        except Exception:
            message = record.getMessage()
        RuntimeLoggingBridge.append_log_to_current_runtime(message)


class RuntimeLoggingBridge:
    """
    Global bridge between logging sources and the currently active runtime.

    The bridge manages three responsibilities:

    - tracking the active runtime instance
    - attaching a Python logging handler to the root logger
    - patching YASMIN logging helpers so their messages are mirrored into the UI
    """

    _current_runtime_ref: Optional[weakref.ReferenceType] = None
    _python_log_handler: Optional[_RuntimeLogHandler] = None
    _root_logger: Optional[logging.Logger] = None
    _original_root_level: Optional[int] = None
    _install_lock = threading.Lock()

    @classmethod
    def set_current_runtime(cls, runtime: Optional[Any]) -> None:
        """
        Register the runtime that should receive forwarded log messages.

        A weak reference is used so that the bridge does not keep the runtime
        alive after shutdown.

        Args:
            runtime: Active runtime instance or ``None`` to clear the target.
        """
        cls._current_runtime_ref = None if runtime is None else weakref.ref(runtime)

    @classmethod
    def get_current_runtime(cls) -> Optional[Any]:
        """
        Return the currently active runtime instance.

        The method resolves the stored weak reference and automatically clears
        it if the referenced runtime was already garbage-collected.

        Returns:
            The active runtime instance, or ``None`` if no valid runtime exists.
        """
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
        """
        Append a formatted log message to the active runtime.

        Args:
            message: Final message text to be stored in the runtime log buffer.
        """
        runtime = cls.get_current_runtime()
        if runtime is not None:
            runtime._append_log(message)

    @classmethod
    def install(cls) -> None:
        """
        Install Python logging forwarding and patch YASMIN logging helpers.

        The operation is thread-safe and idempotent. Repeated calls keep the
        existing handler and skip already wrapped YASMIN functions.
        """
        with cls._install_lock:
            cls._install_python_handler()
            for module in cls._iter_yasmin_modules():
                cls._patch_yasmin_module(module)
        cls.activate_yasmin_logger()

    @classmethod
    def uninstall(cls) -> None:
        """
        Remove the Python logging handler from the root logger.

        YASMIN monkeypatches remain installed for the lifetime of the process,
        but after the current runtime is cleared they no longer forward messages
        anywhere. The root logger level is restored to the value captured during
        installation.
        """
        with cls._install_lock:
            if cls._root_logger is None or cls._python_log_handler is None:
                return
            if cls._python_log_handler in cls._root_logger.handlers:
                cls._root_logger.removeHandler(cls._python_log_handler)
            if cls._original_root_level is not None:
                cls._root_logger.setLevel(cls._original_root_level)
                cls._original_root_level = None

    @classmethod
    def _install_python_handler(cls) -> None:
        """
        Attach the runtime forwarding handler to the process root logger.

        The previous root logger level is stored once so that it can be restored
        during ``uninstall()``. The logger is lowered to ``NOTSET`` to ensure
        that all records reach the installed handler.
        """
        if cls._python_log_handler is None:
            cls._python_log_handler = _RuntimeLogHandler()

        if cls._root_logger is None:
            cls._root_logger = logging.getLogger()

        if cls._python_log_handler not in cls._root_logger.handlers:
            cls._root_logger.addHandler(cls._python_log_handler)

        if cls._original_root_level is None:
            cls._original_root_level = cls._root_logger.level

        if cls._root_logger.level > logging.NOTSET:
            cls._root_logger.setLevel(logging.NOTSET)

    @classmethod
    def _iter_yasmin_modules(cls) -> tuple[Any, ...]:
        """
        Resolve YASMIN modules that may expose logging helpers.

        Some environments expose the helpers from ``yasmin.logs`` while others
        re-export them from ``yasmin``. This method returns all available unique
        modules so the bridge can patch whichever variant exists.

        Returns:
            Tuple of imported YASMIN-related modules.
        """
        module_names = ("yasmin.logs", "yasmin")
        modules: list[Any] = []
        seen: set[int] = set()

        for module_name in module_names:
            try:
                module = importlib.import_module(module_name)
            except Exception:
                continue

            module_id = id(module)
            if module_id in seen:
                continue
            seen.add(module_id)
            modules.append(module)

        return tuple(modules)

    @classmethod
    def _patch_yasmin_module(cls, module: Any) -> None:
        """
        Patch all relevant logging entry points of a YASMIN module.

        The patched module forwards its messages to the runtime while preserving
        the original behavior of the wrapped functions.

        Args:
            module: Imported YASMIN module exposing logging helpers.
        """
        cls._patch_log_function(module, "log_error", "ERROR")
        cls._patch_log_function(module, "log_warn", "WARN")
        cls._patch_log_function(module, "log_info", "INFO")
        cls._patch_log_function(module, "log_debug", "DEBUG")
        cls._patch_set_loggers(module)
        cls._patch_reset_function(module, "set_default_loggers")
        cls._patch_reset_function(module, "set_py_loggers")

    @classmethod
    def _patch_log_function(cls, module: Any, func_name: str, level_name: str) -> None:
        """
        Wrap a YASMIN convenience log function.

        The wrapper mirrors the message into the runtime and then calls the
        original YASMIN function unchanged.

        Args:
            module: Imported YASMIN module.
            func_name: Name of the function to patch.
            level_name: Human-readable log level label used in the editor log.
        """
        original = getattr(module, func_name, None)
        if not callable(original) or getattr(original, _WRAPPER_ATTR, False):
            return

        @functools.wraps(original)
        def wrapped(
            file: str,
            function: str,
            line: int,
            text: str,
            *,
            _original: Callable[..., Any] = original,
            _level_name: str = level_name,
        ) -> Any:
            """
            Forward a YASMIN log call to the runtime before delegating.

            Args:
                file: Source file name reported by YASMIN.
                function: Function name reported by YASMIN.
                line: Source line reported by YASMIN.
                text: Final log message text.

            Returns:
                The return value of the original YASMIN function.
            """
            cls.append_log_to_current_runtime(
                f"[{_level_name}] [{file}:{function}:{line}] {text}"
            )
            return _original(file, function, line, text)

        setattr(wrapped, _WRAPPER_ATTR, True)
        setattr(module, func_name, wrapped)

    @classmethod
    def _patch_set_loggers(cls, module: Any) -> None:
        """
        Wrap YASMIN's logger registration function.

        The wrapper ensures that runtime logging remains active even if external
        code installs its own YASMIN logger callback.

        Args:
            module: Imported YASMIN module.
        """
        original = getattr(module, "set_loggers", None)
        if not callable(original) or getattr(original, _WRAPPER_ATTR, False):
            return

        @functools.wraps(original)
        def wrapped_set_loggers(
            user_logger: Callable[..., Any],
            *,
            _original: Callable[..., Any] = original,
        ) -> Any:
            """
            Install a combined YASMIN logger callback.

            If the requested callback is already the runtime logger, it is passed
            through unchanged. Otherwise, a composite callback is installed that
            first forwards to the runtime and then invokes the user callback.

            Args:
                user_logger: Caller-provided YASMIN logger callback.

            Returns:
                The return value of the original ``set_loggers`` function.
            """
            if user_logger is cls.yasmin_log_message:
                return _original(cls.yasmin_log_message)

            def combined_logger(
                level: Any,
                file: str,
                function: str,
                line: int,
                text: str,
            ) -> None:
                """
                Forward a YASMIN message to both the runtime and the caller.

                Args:
                    level: YASMIN log level.
                    file: Source file reported by YASMIN.
                    function: Function name reported by YASMIN.
                    line: Source line reported by YASMIN.
                    text: Final log message text.
                """
                cls.yasmin_log_message(level, file, function, line, text)
                user_logger(level, file, function, line, text)

            return _original(combined_logger)

        setattr(wrapped_set_loggers, _WRAPPER_ATTR, True)
        setattr(module, "set_loggers", wrapped_set_loggers)

    @classmethod
    def _patch_reset_function(cls, module: Any, func_name: str) -> None:
        """
        Wrap a YASMIN logger reset/helper function.

        Some YASMIN helper functions replace the currently active logger setup.
        After such a reset, the runtime logger is installed again.

        Args:
            module: Imported YASMIN module.
            func_name: Name of the reset/helper function to patch.
        """
        original = getattr(module, func_name, None)
        if not callable(original) or getattr(original, _WRAPPER_ATTR, False):
            return

        @functools.wraps(original)
        def wrapped(
            *args: Any,
            _original: Callable[..., Any] = original,
            **kwargs: Any,
        ) -> Any:
            """
            Execute the original reset function and restore runtime logging.

            Args:
                *args: Positional arguments forwarded to the original function.
                **kwargs: Keyword arguments forwarded to the original function.

            Returns:
                The return value of the original reset/helper function.
            """
            result = _original(*args, **kwargs)
            cls.activate_yasmin_logger()
            return result

        setattr(wrapped, _WRAPPER_ATTR, True)
        setattr(module, func_name, wrapped)

    @classmethod
    def activate_yasmin_logger(cls) -> None:
        """
        Install the runtime logger callback into the available YASMIN module.

        The method searches the discovered YASMIN modules for a callable
        ``set_loggers`` function and registers ``yasmin_log_message`` as the
        active callback.
        """
        set_loggers = None
        for module in cls._iter_yasmin_modules():
            candidate = getattr(module, "set_loggers", None)
            if callable(candidate):
                set_loggers = candidate
                break

        if callable(set_loggers):
            try:
                set_loggers(cls.yasmin_log_message)
            except Exception:
                pass

    @classmethod
    def log_level_to_name(cls, level: Any) -> str:
        """
        Convert a YASMIN log level value into a readable level name.

        The method first tries YASMIN's own conversion helper. If it is not
        available, a small fallback mapping is used.

        Args:
            level: YASMIN log level object, enum, integer, or string.

        Returns:
            Human-readable log level name.
        """
        for module in cls._iter_yasmin_modules():
            log_level_to_name = getattr(module, "log_level_to_name", None)
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
        """
        Default YASMIN logger callback used by the editor runtime.

        Args:
            level: YASMIN log level.
            file: Source file reported by YASMIN.
            function: Function name reported by YASMIN.
            line: Source line reported by YASMIN.
            text: Final log message text.
        """
        cls.append_log_to_current_runtime(
            f"[{cls.log_level_to_name(level)}] [{file}:{function}:{line}] {text}"
        )
