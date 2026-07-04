# Copyright (C) 2026 Maik Knof
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
