# Copyright (C) 2026 Maik Knof
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
from rclpy.serialization import deserialize_message
from yasmin import Blackboard, State

from yasmin_ros.ros_interface_utils import resolve_interface_type


class RosDeserializePyState(State):
    def __init__(self) -> None:
        super().__init__(["succeed", "type_error", "error"])
        self._interface_type = "geometry_msgs/msg/Pose"
        self._interface_cls = None
        self.set_description(
            "Deserializes a byte array from blackboard key 'input' into a Python "
            "ROS interface instance written to blackboard key 'output'."
        )
        self.set_outcome_description("succeed", "Deserialization finished successfully.")
        self.set_outcome_description(
            "type_error",
            "The value stored in 'input' is not a byte array or the target type does not match.",
        )
        self.set_outcome_description(
            "error",
            "Deserialization failed because the serialized data could not be decoded.",
        )
        self.add_input_key(
            "input",
            "Serialized byte array that should be converted back into a ROS interface instance.",
        )
        self.add_output_key(
            "output",
            "Deserialized ROS interface instance in native Python form.",
        )
        self.declare_parameter(
            "interface_type",
            "ROS interface type string such as geometry_msgs/msg/Pose or "
            "example_interfaces/action/Fibonacci_Result.",
            "geometry_msgs/msg/Pose",
        )

    def configure(self) -> None:
        self._interface_type = self.get_parameter("interface_type")
        self._interface_cls = resolve_interface_type(self._interface_type)

    def execute(self, blackboard: Blackboard) -> str:
        try:
            serialized_data = blackboard["input"]
            if not isinstance(serialized_data, (bytes, bytearray, memoryview)):
                yasmin.YASMIN_LOG_WARN(
                    f"RosDeserializePyState expected bytes for '{self._interface_type}' but received '{type(serialized_data).__name__}'"
                )
                return "type_error"

            blackboard["output"] = deserialize_message(
                bytes(serialized_data), self._interface_cls
            )
            return "succeed"
        except Exception as exc:
            yasmin.YASMIN_LOG_WARN(
                f"RosDeserializePyState failed for '{self._interface_type}': {exc}"
            )
            return "error"
