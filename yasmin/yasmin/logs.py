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
from typing import Callable, List, Union

import yasmin

__all__ = [
    "LogLevel",
    "log_level",
    "set_log_level",
    "log_level_to_name",
    "log_message",
    "YASMIN_LOG_ERROR",
    "YASMIN_LOG_WARN",
    "YASMIN_LOG_INFO",
    "YASMIN_LOG_DEBUG",
    "set_loggers",
    "set_default_loggers",
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
log_level = LogLevel.INFO


## Sets the log level for the logs.
def set_log_level(level: LogLevel) -> None:
    """
    @brief Set the log level for the YASMIN framework.

    Adjusts the log level to control the verbosity of logged messages.

    @param level The new log level to be set.
    """
    yasmin.log_level = level


def log_level_to_name(level: LogLevel) -> str:

    if level == LogLevel.ERROR:
        return "ERROR"
    elif level == LogLevel.WARN:
        return "WARN"
    elif level == LogLevel.INFO:
        return "INFO"
    elif level == LogLevel.DEBUG:
        return "DEBUG"

    return ""


def get_caller_info() -> List[Union[str, int]]:
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


def default_log_message(
    level: LogLevel, file: str, function: str, line: int, text: str
) -> None:
    """
    Default logging function.

    This function is the default logging function of YASMIN.

    @param level The log level as a string (e.g., "ERROR", "WARN", "INFO",
    "DEBUG").
    @param file The source file where the log function is called.
    @param function The function where the log function is called.
    @param line The line number in the source file.
    @param text The format string for the log message.
    @param args Additional arguments for the format string.
    """

    message = f"[{log_level_to_name(level)}] [{file}:{function}:{line}] {text}"

    if level == LogLevel.ERROR:
        logging.error(message)

    elif level == LogLevel.WARN:
        logging.warning(message)

    elif level == LogLevel.INFO:
        logging.info(message)

    elif level == LogLevel.DEBUG:
        logging.debug(message)


log_message = default_log_message


def log_helper(level: LogLevel, file: str, function: str, line: int, text: str) -> None:
    """
    @brief Variadic template function to log messages at different levels.

    This function wraps log_message and allows logging messages with different
    log levels while reducing redundant code. It provides a consistent logging
    format across all levels.

    @tparam LEVEL The log level LogLevel (e.g., 0 -> "ERROR", 1 -> "WARN", 2 ->
    "INFO", 3 -> "DEBUG").
    @param log_message Function to create the logs
    @param file The source file where the log function is called.
    @param function The function where the log function is called.
    @param line The line number in the source file.
    @param text The format string for the log message.
    @param ... Additional arguments for the format string.
    """

    if yasmin.log_level >= level:
        yasmin.log_message(level, file, function, line, text)


def YASMIN_LOG_ERROR(text: str) -> None:
    """
    Log an error message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The error message to log.
    @type text: str

    @return: None
    """
    file, function, line = get_caller_info()
    log_helper(LogLevel.ERROR, file, function, line, text)


def YASMIN_LOG_WARN(text: str) -> None:
    """
    Log a warning message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The warning message to log.
    @type text: str

    @return: None
    """
    file, function, line = get_caller_info()
    log_helper(LogLevel.WARN, file, function, line, text)


def YASMIN_LOG_INFO(text: str) -> None:
    """
    Log an informational message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The informational message to log.
    @type text: str

    @return: None
    """
    file, function, line = get_caller_info()
    log_helper(LogLevel.INFO, file, function, line, text)


def YASMIN_LOG_DEBUG(text: str) -> None:
    """
    Log a debug message with the caller's information.

    This function formats the log message to include the file name, function
    name, and line number where the log function was called.

    @param text: The debug message to log.
    @type text: str

    @return: None
    """
    file, function, line = get_caller_info()
    log_helper(LogLevel.DEBUG, file, function, line, text)


def set_loggers(new_log_message: Callable[[LogLevel, str], None]) -> None:
    """
    Set custom logger functions for YASMIN logging.

    This function assigns user-defined logging function for different log
    levels: info, warning, debug, and error.

    @param log_message: A callable function for logging messages.
    @type info: Callable[[LogLevel, str], None]

    @return: None

    @raises TypeError: If any of the parameters are not callable.
    """

    if not callable(new_log_message):
        raise TypeError("log_message must be callable.")

    yasmin.log_message = new_log_message


def set_default_loggers() -> None:
    """
    Sets the default logging function for all log levels.

    This function initializes the logging function to the default
    implementations.
    """
    set_loggers(log_message)
