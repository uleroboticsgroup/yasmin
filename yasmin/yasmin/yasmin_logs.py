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

import yasmin
import logging

import yasmin.yasmin_logs

__all__ = [
    "set_loggers",
    "YASMIN_LOG_ERROR",
    "YASMIN_LOG_WARN",
    "YASMIN_LOG_INFO",
    "YASMIN_LOG_DEBUG",
]


# Define the logging configuration
logging.basicConfig(level=logging.NOTSET, format="%(message)s")


def YASMIN_LOG_ERROR(text: str) -> None:
    logging.error("[ERROR] " + text)


def YASMIN_LOG_WARN(text: str) -> None:
    logging.warning("[WARN] " + text)


def YASMIN_LOG_INFO(text: str) -> None:
    logging.info("[INFO] " + text)


def YASMIN_LOG_DEBUG(text: str) -> None:
    logging.debug("[DEBUG] " + text)


def set_loggers(info, warn, debug, error):
    yasmin.YASMIN_LOG_ERROR = error
    yasmin.YASMIN_LOG_WARN = warn
    yasmin.YASMIN_LOG_INFO = info
    yasmin.YASMIN_LOG_DEBUG = debug
