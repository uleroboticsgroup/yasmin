# Copyright (C) 2024 Miguel Ángel González Santamarta
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


from threading import Lock

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode

__all__ = [
    "set_ros_loggers",
    "logger_node",
]

## Node used in the YASMIN ROS 2 logs
logger_node: Node = None
## Lock protecting logger_node access
_logger_node_lock: Lock = Lock()


def ros_log_message(
    level: yasmin.logs.LogLevel, file: str, function: str, line: int, text: str
) -> None:
    """
    Logs a message to the ROS 2 logger.

    This function logs a message using a ROS 2 logger.

    @param level The log level (ERROR, WARN, INFO, DEBUG).
    @param file The source file where the log function is called.
    @param function The function where the log function is called.
    @param line The line number in the source file.
    @param text The message to log.
    """

    message = f"[{file}:{function}:{line}] {text}"

    with _logger_node_lock:
        current_logger_node = yasmin_ros.logger_node

    logger = (
        current_logger_node.get_logger()
        if current_logger_node is not None
        else rclpy.logging.get_logger("yasmin_ros")
    )

    if level == yasmin.logs.LogLevel.ERROR:
        logger.error(message)

    elif level == yasmin.logs.LogLevel.WARN:
        logger.warning(message)

    elif level == yasmin.logs.LogLevel.INFO:
        logger.info(message)

    elif level == yasmin.logs.LogLevel.DEBUG:
        logger.debug(message)


def set_ros_loggers(node: Node = None) -> None:
    """
    Sets the ROS loggers for various logging levels.

    This function assigns the logging functions for info, warning,
    debug, and error messages to the Yasmin logging framework.

    @param node: ROS 2 node to use as logger node. If None, use YasminNode.
    @type node: Node
    """

    with _logger_node_lock:
        if node is None:
            yasmin_ros.logger_node = YasminNode.get_instance()
        else:
            yasmin_ros.logger_node = node

    yasmin.set_loggers(ros_log_message)
