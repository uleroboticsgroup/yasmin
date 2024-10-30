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
import yasmin.logs
from yasmin_ros.yasmin_node import YasminNode


__all__ = ["set_ros_loggers"]


def ros_log_error(text: str) -> None:
    file, function, line = yasmin.logs.get_caller_info()
    node = YasminNode.get_instance()
    node.get_logger().error(f"[{file}:{function}:{line}] {text}")


def ros_log_warn(text: str) -> None:
    file, function, line = yasmin.logs.get_caller_info()
    node = YasminNode.get_instance()
    node.get_logger().warn(f"[{file}:{function}:{line}] {text}")


def ros_log_info(text: str) -> None:
    file, function, line = yasmin.logs.get_caller_info()
    node = YasminNode.get_instance()
    node.get_logger().info(f"[{file}:{function}:{line}] {text}")


def ros_log_debug(text: str) -> None:
    file, function, line = yasmin.logs.get_caller_info()
    node = YasminNode.get_instance()
    node.get_logger().debug(f"[{file}:{function}:{line}] {text}")


def set_ros_loggers() -> None:
    yasmin.set_loggers(
        ros_log_info,
        ros_log_warn,
        ros_log_debug,
        ros_log_error,
    )
