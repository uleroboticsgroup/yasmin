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

"""Logging utilities for the YASMIN editor runtime.

This module encapsulates the runtime log buffer, YASMIN logger integration,
tree-style indentation for nested containers, and duplicate suppression.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, List, Optional, Union

import yasmin

LOG_LEVEL_BY_NAME = {
    "ERROR": yasmin.LogLevel.ERROR,
    "WARN": yasmin.LogLevel.WARN,
    "INFO": yasmin.LogLevel.INFO,
    "DEBUG": yasmin.LogLevel.DEBUG,
}


class RuntimeLogger:
    """Bridge YASMIN logging into the editor runtime log view.

    The logger stores runtime log lines in memory, forwards them to Qt via
    callbacks, and keeps track of visual depth for nested container execution.
    """

    def __init__(
        self,
        append_callback: Callable[[str], None],
        clear_callback: Callable[[], None],
        is_disposed: Callable[[], bool],
    ) -> None:
        """Initialize the runtime logger bridge.

        Args:
            append_callback: Callback used to forward one log line to the UI.
            clear_callback: Callback used to notify the UI that logs were reset.
            is_disposed: Predicate indicating whether the owning runtime is
                already disposed and should ignore new log traffic.
        """
        self._append_callback = append_callback
        self._clear_callback = clear_callback
        self._is_disposed = is_disposed

        self._log_entries: List[str] = []
        self._log_buffer_lock = threading.Lock()
        self._last_log_message = ""
        self._last_log_timestamp = 0.0

        self._log_level = yasmin.LogLevel.INFO
        self._log_depth = 0
        self._log_depth_lock = threading.Lock()

        self.configure()

    def get_logs(self) -> List[str]:
        """Return a copy of the collected runtime log lines."""
        with self._log_buffer_lock:
            return list(self._log_entries)

    def get_log_level_name(self) -> str:
        """Return the active YASMIN log level as an uppercase string."""
        return str(yasmin.log_level_to_name(self._log_level)).upper()

    def set_log_level(
        self,
        level: Union[yasmin.LogLevel, str],
        emit_status: bool = True,
    ) -> None:
        """Update the active YASMIN log level.

        Args:
            level: Target log level as a ``yasmin.LogLevel`` or its name.
            emit_status: Whether the log view should receive a status line.
        """
        if isinstance(level, str):
            normalized_level = LOG_LEVEL_BY_NAME.get(level.strip().upper())
            if normalized_level is None:
                raise ValueError(f"Unsupported log level: {level}")
            level = normalized_level

        self._log_level = level
        self.configure()

        if emit_status:
            self.append(f"[STATUS] Log level set to {self.get_log_level_name()}")

    def configure(self) -> None:
        """Install the runtime logger callback into YASMIN."""
        yasmin.set_loggers(self._handle_yasmin_log)
        yasmin.set_log_level(self._log_level)

    def clear(self) -> None:
        """Reset the in-memory runtime log buffer."""
        with self._log_buffer_lock:
            self._log_entries.clear()
            self._last_log_message = ""
            self._last_log_timestamp = 0.0
        self._clear_callback()

    def reset_depth(self) -> None:
        """Reset the visual log depth to the top level."""
        with self._log_depth_lock:
            self._log_depth = 0

    def increment_depth(self) -> None:
        """Increase the visual log depth after entering a container."""
        with self._log_depth_lock:
            self._log_depth += 1

    def decrement_depth(self) -> None:
        """Decrease the visual log depth after leaving a container."""
        with self._log_depth_lock:
            self._log_depth = max(0, self._log_depth - 1)

    def get_depth(self) -> int:
        """Return the current visual log depth."""
        with self._log_depth_lock:
            return self._log_depth

    def append(
        self,
        message: str,
        depth: Optional[int] = None,
        is_end: bool = False,
    ) -> None:
        """Append a log line while filtering accidental duplicate bursts.

        Args:
            message: Text to append to the runtime log buffer.
            depth: Optional explicit nesting depth. If omitted, the currently
                tracked visual depth is used.
            is_end: Whether the line should use the visual end-branch marker.
        """
        if self._is_disposed():
            return

        clean_message = str(message).rstrip()
        if not clean_message:
            return

        current_depth = self.get_depth() if depth is None else max(0, depth)
        if current_depth > 0:
            clean_message = (
                f"{self._format_log_tree_prefix(current_depth, is_end=is_end)}"
                f"{clean_message}"
            )

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

        self._append_callback(clean_message)

    def _handle_yasmin_log(
        self,
        level: Any,
        file: str,
        function: str,
        line: int,
        text: str,
    ) -> None:
        """Forward YASMIN logs to the runtime log buffer."""
        message = (
            f"[{yasmin.log_level_to_name(level)}] " f"[{file}:{function}:{line}] {text}"
        )
        self.append(message)

    def _format_log_tree_prefix(self, depth: int, is_end: bool = False) -> str:
        """Return a tree-style prefix for nested runtime log lines."""
        if depth <= 0:
            return ""
        branch = "└─ " if is_end else "├─ "
        return ("│  " * (depth - 1)) + branch
