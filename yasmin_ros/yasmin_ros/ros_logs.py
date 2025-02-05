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


from rclpy.node import Node

import yasmin
import yasmin.logs
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode

__all__ = [
    "set_ros_loggers",
    "logger_node",
]

## Node used in the YASMIN ROS 2 logs
logger_node: Node = None


def ros_log_message(
    level: yasmin.logs.LogLevel, file: str, function: str, line: int, text: str
) -> None:
    """
    Logs a message to the ROS 2 logger.

    This function logs a message using a ROS 2 logger.

    @param text: The debug message to log.
    @type text: str

    @raises: None

    @return: None
    """

    message = f"[{file}:{function}:{line}] {text}"

    if level == yasmin.logs.LogLevel.ERROR:
        yasmin_ros.logger_node.get_logger().error(message)

    elif level == yasmin.logs.LogLevel.WARN:
        yasmin_ros.logger_node.get_logger().warn(message)

    elif level == yasmin.logs.LogLevel.INFO:
        yasmin_ros.logger_node.get_logger().info(message)

    elif level == yasmin.logs.LogLevel.DEBUG:
        yasmin_ros.logger_node.get_logger().debug(message)


def set_ros_loggers(node: Node = None) -> None:
    """
    Sets the ROS loggers for various logging levels.

    This function assigns the logging functions for info, warning,
    debug, and error messages to the Yasmin logging framework.

    @param node: ROS 2 node to use as logger node. If None, use YasminNode.
    @type node: Node

    @raises: None

    @return: None
    """

    if node is None:
        yasmin_ros.logger_node = YasminNode.get_instance()
    else:
        yasmin_ros.logger_node = node

    yasmin.set_loggers(ros_log_message)
