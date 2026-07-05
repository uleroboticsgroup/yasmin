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

from typing import Dict, Any

import rclpy
from rclpy.node import Node

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.ros_state_utils import resolve_node


class GetParametersState(State):
    """
    State that retrieves parameters from the ROS 2 parameter server.

    This state retrieves parameters from the ROS 2 parameter server and stores
    them in the blackboard.
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
        self._node = resolve_node(node)

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
            if self.is_canceled():
                return CANCEL

            if not self._node.has_parameter(param_name):
                self._node.declare_parameter(param_name, param_value)

            yasmin.YASMIN_LOG_INFO(f"Retrieving parameter '{param_name}'")

            parameter = self._node.get_parameter(param_name)
            parameter_type = rclpy.Parameter.Type(
                self._node.get_parameter_type(param_name)
            )
            parameter_value = parameter.get_parameter_value()

            if parameter_type == rclpy.Parameter.Type.BOOL:
                value = parameter_value.bool_value
            elif parameter_type == rclpy.Parameter.Type.INTEGER:
                value = parameter_value.integer_value
            elif parameter_type == rclpy.Parameter.Type.DOUBLE:
                value = parameter_value.double_value
            elif parameter_type == rclpy.Parameter.Type.STRING:
                value = parameter_value.string_value
            elif parameter_type == rclpy.Parameter.Type.BOOL_ARRAY:
                value = parameter_value.bool_array_value
            elif parameter_type == rclpy.Parameter.Type.INTEGER_ARRAY:
                value = parameter_value.integer_array_value
            elif parameter_type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                value = parameter_value.double_array_value
            elif parameter_type == rclpy.Parameter.Type.STRING_ARRAY:
                value = parameter_value.string_array_value
            elif parameter_type == rclpy.Parameter.Type.BYTE_ARRAY:
                value = parameter_value.byte_array_value
            else:
                yasmin.YASMIN_LOG_ERROR(
                    f"Unsupported parameter type for '{param_name}': {self._node.get_parameter_type(param_name)}"
                )
                return ABORT

            blackboard[param_name] = value

        return SUCCEED
