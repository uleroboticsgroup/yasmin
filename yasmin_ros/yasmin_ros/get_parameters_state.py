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
    GetParametersState is a state that retrieves ROS 2 parameters.

    Attributes:
        _node (Node): The ROS 2 node instance used for subscriptions.
    """

    def __init__(
        self,
        parameters: Dict[str, Any],
        node: Node = None,
    ) -> None:
        """
        Initializes the GetParametersState.

        Parameters:
            parameters (Dict[str, Any]): The parameters names to retrieve and their default values.
            node (Node, optional): The ROS node to use. If None, a default node is created.

        Returns:
            None
        """
        self._parameters = parameters
        self._node = node if node else YasminNode.get_instance()

        super().__init__([SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the state.

        This method retrieves the specified parameters from the ROS 2 node.

        Parameters:
            blackboard (Blackboard): The blackboard instance that holds shared data.

        Returns:
            str: The outcome of the state execution.
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
