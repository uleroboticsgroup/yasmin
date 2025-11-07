# Copyright (C) 2025 Miguel Ángel González Santamarta
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

"""Python bindings for yasmin logging"""

from enum import Enum
from typing import Callable

class LogLevel(Enum):
    """
    Enumeration for logging levels.

    Attributes:
        ERROR: Log level for error messages. Only critical errors should be logged.
        WARN: Log level for warning messages. Indicate potential issues that are not critical.
        INFO: Log level for informational messages. General runtime information.
        DEBUG: Log level for debug messages. Detailed information for developers.
    """

    ERROR: int
    WARN: int
    INFO: int
    DEBUG: int

def get_log_level() -> LogLevel:
    """
    Get the current log level.

    Returns:
        The current log level
    """
    ...

def set_log_level(level: LogLevel) -> None:
    """
    Set the log level for the YASMIN framework.

    Args:
        level: The log level to set
    """
    ...

def log_level_to_name(level: LogLevel) -> str:
    """
    Convert a log level to its string name.

    Args:
        level: The log level to convert

    Returns:
        String name of the log level (e.g., "ERROR", "WARN", "INFO", "DEBUG")
    """
    ...

def log_error(file: str, function: str, line: int, text: str) -> None:
    """
    Log an error message.

    Args:
        file: Source file where the log is called from
        function: Function name where the log is called from
        line: Line number where the log is called from
        text: The message to log
    """
    ...

def log_warn(file: str, function: str, line: int, text: str) -> None:
    """
    Log a warning message.

    Args:
        file: Source file where the log is called from
        function: Function name where the log is called from
        line: Line number where the log is called from
        text: The message to log
    """
    ...

def log_info(file: str, function: str, line: int, text: str) -> None:
    """
    Log an info message.

    Args:
        file: Source file where the log is called from
        function: Function name where the log is called from
        line: Line number where the log is called from
        text: The message to log
    """
    ...

def log_debug(file: str, function: str, line: int, text: str) -> None:
    """
    Log a debug message.

    Args:
        file: Source file where the log is called from
        function: Function name where the log is called from
        line: Line number where the log is called from
        text: The message to log
    """
    ...

def set_loggers(log_function: Callable[[LogLevel, str, str, int, str], None]) -> None:
    """
    Set a custom logging function.

    The function should accept the following parameters:
    - level: LogLevel - The severity level of the log message
    - file: str - Source file where the log is called from
    - function: str - Function name where the log is called from
    - line: int - Line number where the log is called from
    - text: str - The message to log

    Args:
        log_function: Custom logging function with the signature above
    """
    ...

def set_default_loggers() -> None:
    """
    Reset to the default logging function.

    This restores the built-in C++ logging implementation.
    """
    ...
