# Copyright (C) 2025 Miguel Ángel González Santamarta
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
