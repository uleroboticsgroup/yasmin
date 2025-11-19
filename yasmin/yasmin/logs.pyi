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

from enum import Enum
from typing import Callable

class LogLevel(Enum):
    ERROR: int
    WARN: int
    INFO: int
    DEBUG: int

def get_log_level() -> LogLevel: ...
def set_log_level(level: LogLevel) -> None: ...
def log_level_to_name(level: LogLevel) -> str: ...
def log_error(file: str, function: str, line: int, text: str) -> None: ...
def log_warn(file: str, function: str, line: int, text: str) -> None: ...
def log_info(file: str, function: str, line: int, text: str) -> None: ...
def log_debug(file: str, function: str, line: int, text: str) -> None: ...
def set_loggers(log_function: Callable[[LogLevel, str, str, int, str], None]) -> None: ...
def set_default_loggers() -> None: ...
