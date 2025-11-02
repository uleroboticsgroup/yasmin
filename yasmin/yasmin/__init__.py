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
from typing import List, Union

from yasmin.state import State
from yasmin.concurrence import Concurrence
from yasmin.cb_state import CbState
from yasmin.blackboard import Blackboard
from yasmin.state_machine import StateMachine
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
    file, function, line = get_caller_info()
    log_error(file, function, line, text)


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
    log_warn(file, function, line, text)


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
    log_info(file, function, line, text)


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
    log_debug(file, function, line, text)
