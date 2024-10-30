# Copyright (C) 2024  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import yasmin
import logging
import inspect
from typing import Callable

__all__ = [
    "set_loggers",
    "YASMIN_LOG_ERROR",
    "YASMIN_LOG_WARN",
    "YASMIN_LOG_INFO",
    "YASMIN_LOG_DEBUG",
    "get_caller_info",
]

# define the logging configuration with custom format to include location data
logging.basicConfig(level=logging.NOTSET, format="%(message)s")


def get_caller_info():
    frame = inspect.stack()[2]
    file = os.path.basename(frame.filename)
    line = frame.lineno
    function = frame.function
    return file, function, line


def YASMIN_LOG_ERROR(text: str) -> None:
    file, function, line = get_caller_info()
    logging.error(f"[ERROR] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_WARN(text: str) -> None:
    file, function, line = get_caller_info()
    logging.warning(f"[WARN] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_INFO(text: str) -> None:
    file, function, line = get_caller_info()
    logging.info(f"[INFO] [{file}:{function}:{line}] {text}")


def YASMIN_LOG_DEBUG(text: str) -> None:
    file, function, line = get_caller_info()
    logging.debug(f"[DEBUG] [{file}:{function}:{line}] {text}")


def set_loggers(
    info: Callable,
    warn: Callable,
    debug: Callable,
    error: Callable,
) -> None:
    yasmin.YASMIN_LOG_ERROR = error
    yasmin.YASMIN_LOG_WARN = warn
    yasmin.YASMIN_LOG_INFO = info
    yasmin.YASMIN_LOG_DEBUG = debug
