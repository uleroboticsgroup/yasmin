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


import logging

# Define the logging configuration
logging.basicConfig(level=logging.NOTSET, format="%(message)s")


def YASMIN_LOG_ERROR(text: str, *args) -> None:
    logging.error("[ERROR] " + text, *args)


def YASMIN_LOG_WARN(text: str, *args) -> None:
    logging.warning("[WARN] " + text, *args)


def YASMIN_LOG_INFO(text: str, *args) -> None:
    logging.info("[INFO] " + text, *args)


def YASMIN_LOG_DEBUG(text: str, *args) -> None:
    logging.info("[DEBUG] " + text, *args)
