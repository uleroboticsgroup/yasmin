# Copyright (C) 2024 Miguel Ángel González Santamarta
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
import inspect
import logging
from enum import IntEnum
from typing import Callable

import yasmin

__all__ = [
    "LogLevel",
    "set_log_level",
    "log_level",
    "set_loggers",
    "YASMIN_LOG_ERROR",
    "YASMIN_LOG_WARN",
    "YASMIN_LOG_INFO",
    "YASMIN_LOG_DEBUG",
    "get_caller_info",
]

# Configure logging with a custom format to include location data
logging.basicConfig(level=logging.NOTSET, format="%(message)s")


class LogLevel(IntEnum):
    """
    @enum LogLevel
    @brief Enumeration for different log levels.

    Defines the available log levels for controlling verbosity in the
    logging system.
    """

    ## Log level for error messages. Only critical errors should be logged.
    ERROR = 0
    ## Log level for warning messages. Indicate potential issues that are not critical.
    WARN = 1
    ## Log level for informational messages. General runtime information about the system's state.
    INFO = 2
    ## Log level for debug messages. Used for detailed information, mainly for developers.
    DEBUG = 3


## The current log level for the application.
log_level = LogLevel.DEBUG


## Sets the log level for the logs.
def set_log_level(level: LogLevel) -> None:
    """
    @brief Set the log level for the YASMIN framework.

    Adjusts the log level to control the verbosity of logged messages.

    @param level The new log level to be set.
    """
    yasmin.log_level = level


def get_caller_info():
    """
    Retrieve information about the caller of the current function.

    This function inspects the call stack to obtain the file name, function
    name, and line number of the caller.

    @return: A tuple containing the file name, function name, and line number.
    @rtype: tuple[str, str, int]
    """
    frame = inspect.stack()[2]
    file = os.path.basename(frame.filename)
    line = frame.lineno
    function = frame.function
    return file, function, line


def YASMIN_LOG_ERROR(text: str) -> None:
    """
    Log an error message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The error message to log.
    @type text: str

    @return: None
    """
    if yasmin.log_level >= LogLevel.ERROR:
        file, function, line = get_caller_info()
        logging.error(f"[ERROR] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_WARN(text: str) -> None:
    """
    Log a warning message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The warning message to log.
    @type text: str

    @return: None
    """
    if yasmin.log_level >= LogLevel.WARN:
        file, function, line = get_caller_info()
        logging.warning(f"[WARN] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_INFO(text: str) -> None:
    """
    Log an informational message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The informational message to log.
    @type text: str

    @return: None
    """
    if yasmin.log_level >= LogLevel.INFO:
        file, function, line = get_caller_info()
        logging.info(f"[INFO] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_DEBUG(text: str) -> None:
    """
    Log a debug message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The debug message to log.
    @type text: str

    @return: None
    """
    if yasmin.log_level >= LogLevel.DEBUG:
        file, function, line = get_caller_info()
        logging.debug(f"[DEBUG] [{file}:{function}:{line}] {text}")


def set_loggers(
    info: Callable[[str], None],
    warn: Callable[[str], None],
    debug: Callable[[str], None],
    error: Callable[[str], None],
) -> None:
    """
    Set custom logger functions for YASMIN logging.

    This function assigns user-defined logging functions for different log
    levels: info, warning, debug, and error.

    @param info: A callable function for logging informational messages.
    @type info: Callable[[str], None]

    @param warn: A callable function for logging warning messages.
    @type warn: Callable[[str], None]

    @param debug: A callable function for logging debug messages.
    @type debug: Callable[[str], None]

    @param error: A callable function for logging error messages.
    @type error: Callable[[str], None]

    @return: None

    @raises TypeError: If any of the parameters are not callable.
    """
    if not all(callable(func) for func in [info, warn, debug, error]):
        raise TypeError("All logger parameters must be callable.")

    yasmin.YASMIN_LOG_ERROR = error
    yasmin.YASMIN_LOG_WARN = warn
    yasmin.YASMIN_LOG_INFO = info
    yasmin.YASMIN_LOG_DEBUG = debug
