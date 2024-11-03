# Copyright (C) 2024  Miguel Ángel González Santamarta
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


import yasmin
import yasmin.logs
from yasmin_ros.yasmin_node import YasminNode

__all__ = ["set_ros_loggers"]


def ros_log_error(text: str) -> None:
    """
    Logs an error message to the ROS logger.

    This function retrieves the caller's file name, function name,
    and line number and logs an error message with that context.

    @param text: The error message to log.
    @type text: str

    @raises: None

    @return: None
    """
    file, function, line = yasmin.logs.get_caller_info()  # Retrieve caller info
    node = YasminNode.get_instance()  # Get the instance of YasminNode
    node.get_logger().error(f"[{file}:{function}:{line}] {text}")  # Log the error


def ros_log_warn(text: str) -> None:
    """
    Logs a warning message to the ROS logger.

    This function retrieves the caller's file name, function name,
    and line number and logs a warning message with that context.

    @param text: The warning message to log.
    @type text: str

    @raises: None

    @return: None
    """
    file, function, line = yasmin.logs.get_caller_info()  # Retrieve caller info
    node = YasminNode.get_instance()  # Get the instance of YasminNode
    node.get_logger().warn(f"[{file}:{function}:{line}] {text}")  # Log the warning


def ros_log_info(text: str) -> None:
    """
    Logs an informational message to the ROS logger.

    This function retrieves the caller's file name, function name,
    and line number and logs an informational message with that context.

    @param text: The informational message to log.
    @type text: str

    @raises: None

    @return: None
    """
    file, function, line = yasmin.logs.get_caller_info()  # Retrieve caller info
    node = YasminNode.get_instance()  # Get the instance of YasminNode
    node.get_logger().info(f"[{file}:{function}:{line}] {text}")  # Log the info


def ros_log_debug(text: str) -> None:
    """
    Logs a debug message to the ROS logger.

    This function retrieves the caller's file name, function name,
    and line number and logs a debug message with that context.

    @param text: The debug message to log.
    @type text: str

    @raises: None

    @return: None
    """
    file, function, line = yasmin.logs.get_caller_info()  # Retrieve caller info
    node = YasminNode.get_instance()  # Get the instance of YasminNode
    node.get_logger().debug(f"[{file}:{function}:{line}] {text}")  # Log the debug info


def set_ros_loggers() -> None:
    """
    Sets the ROS loggers for various logging levels.

    This function assigns the logging functions for info, warning,
    debug, and error messages to the Yasmin logging framework.

    @raises: None

    @return: None
    """
    yasmin.set_loggers(
        ros_log_info,  # Set info logger
        ros_log_warn,  # Set warning logger
        ros_log_debug,  # Set debug logger
        ros_log_error,  # Set error logger
    )
