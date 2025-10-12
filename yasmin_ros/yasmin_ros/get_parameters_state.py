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

from typing import Dict, Union, Type, Any

import rclpy
from rclpy.node import Node

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class GetParametersState(State):
    """
    State that retrieves parameters from the ROS 2 parameter server.

    This state retrieves parameters from the ROS 2 parameter server and stores
    them in the blackboard.

    Attributes:
        _parameters (Dict[str, Any]): Map of parameters to retrieve, where the key is the parameter name
            and the value is the default value.
        _node (Node): Shared pointer to the ROS 2 node.
    """

    def __init__(
        self,
        parameters: Dict[str, Any],
        node: Node = None,
    ) -> None:
        """
        Constructs a GetParametersState with a map of parameters.

        Args:
            parameters (Dict[str, Any]): A map of parameter names to their default values.
            node (Node, optional): A shared pointer to the ROS 2 node.
        """
        self._parameters = parameters
        self._node = node if node else YasminNode.get_instance()

        super().__init__([SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the state to retrieve parameters.

        Args:
            blackboard (Blackboard): A reference to the Yasmin blackboard.

        Returns:
            str: A string representing the outcome of the execution.
        """

        for param_name, param_value in self._parameters.items():
            if not self._node.has_parameter(param_name):
                self._node.declare_parameter(param_name, param_value)

            yasmin.YASMIN_LOG_INFO(f"Retrieving parameter '{param_name}'")

            parameter = self._node.get_parameter(param_name).get_parameter_value()
            parameter_type = rclpy.Parameter.Type(
                self._node.get_parameter_type(param_name)
            )

            if parameter_type == rclpy.Parameter.Type.BOOL:
                value = parameter.bool_value
            elif parameter_type == rclpy.Parameter.Type.INTEGER:
                value = parameter.integer_value
            elif parameter_type == rclpy.Parameter.Type.DOUBLE:
                value = parameter.double_value
            elif parameter_type == rclpy.Parameter.Type.STRING:
                value = parameter.string_value
            elif parameter_type == rclpy.Parameter.Type.BOOL_ARRAY:
                value = parameter.bool_array_value
            elif parameter_type == rclpy.Parameter.Type.INTEGER_ARRAY:
                value = parameter.integer_array_value
            elif parameter_type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                value = parameter.double_array_value
            elif parameter_type == rclpy.Parameter.Type.STRING_ARRAY:
                value = parameter.string_array_value
            elif parameter_type == rclpy.Parameter.Type.BYTE_ARRAY:
                value = parameter.byte_array_value
            else:
                yasmin.YASMIN_LOG_ERROR(
                    f"Unsupported parameter type for '{param_name}': {self._node.get_parameter_type(param_name)}"
                )
                return ABORT

            blackboard[param_name] = value

        return SUCCEED
