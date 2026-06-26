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

import os
import inspect
import logging
from typing import List, Union

from yasmin.callback_signal import CallbackSignal, CallbackSignalFuture
from yasmin.blackboard import Blackboard
from yasmin.state import State
from yasmin.concurrence import Concurrence
from yasmin.cb_state import CbState
from yasmin.state_machine import StateMachine
from yasmin.join_state import JoinState
from yasmin.orthogonal_state import OrthogonalState
from yasmin.logs import (
    LogLevel,
    get_log_level,
    set_log_level,
    log_level_to_name,
    set_loggers,
    set_default_loggers,
    log_error,
    log_warn,
    log_info,
    log_debug,
)


def get_caller_info() -> List[Union[str, int]]:
    """
    Retrieve information about the caller of the current function.

    This function inspects the call stack to obtain the file name, function
    name, and line number of the caller.

    @return: A tuple containing the file name, function name, and line number.
    @rtype: tuple[str, str, int]
    """
    frame = inspect.currentframe().f_back.f_back
    file = os.path.basename(frame.f_code.co_filename)
    line = frame.f_lineno
    function = frame.f_code.co_name
    return file, function, line


def py_default_log_message(
    level: LogLevel, file: str, function: str, line: int, text: str
) -> None:
    """
    Default python logging function.

    @param level The log level as a string (e.g., "ERROR", "WARN", "INFO",
    "DEBUG").
    @param file The source file where the log function is called.
    @param function The function where the log function is called.
    @param line The line number in the source file.
    @param text The format string for the log message.
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


def set_py_loggers() -> None:
    """
    Set the Python logging function for YASMIN.
    """
    logging.basicConfig(level=logging.NOTSET, format="%(message)s")
    set_loggers(py_default_log_message)


def _make_log_wrapper(log_func, name):
    def wrapper(text: str) -> None:
        file, function, line = get_caller_info()
        log_func(file, function, line, str(text))

    wrapper.__name__ = name
    return wrapper


YASMIN_LOG_ERROR = _make_log_wrapper(log_error, "YASMIN_LOG_ERROR")
YASMIN_LOG_WARN = _make_log_wrapper(log_warn, "YASMIN_LOG_WARN")
YASMIN_LOG_INFO = _make_log_wrapper(log_info, "YASMIN_LOG_INFO")
YASMIN_LOG_DEBUG = _make_log_wrapper(log_debug, "YASMIN_LOG_DEBUG")


__all__ = [
    "State",
    "Concurrence",
    "JoinState",
    "OrthogonalState",
    "CbState",
    "CallbackSignal",
    "CallbackSignalFuture",
    "Blackboard",
    "StateMachine",
    "get_log_level",
    "set_log_level",
    "log_level_to_name",
    "set_loggers",
    "set_default_loggers",
    "set_py_loggers",
    "YASMIN_LOG_ERROR",
    "YASMIN_LOG_WARN",
    "YASMIN_LOG_INFO",
    "YASMIN_LOG_DEBUG",
]
