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
from rclpy.serialization import serialize_message
from yasmin import Blackboard, State

from yasmin_ros.ros_interface_utils import resolve_interface_type


class RosSerializePyState(State):
    def __init__(self) -> None:
        super().__init__(["succeed", "type_error", "error"])
        self._interface_type = "geometry_msgs/msg/Pose"
        self._interface_cls = None
        self.set_description(
            "Serializes a Python ROS interface instance from blackboard key "
            "'input' into a byte array written to blackboard key 'output'."
        )
        self.set_outcome_description("succeed", "Serialization finished successfully.")
        self.set_outcome_description(
            "type_error",
            "The value stored in 'input' does not match the configured interface.",
        )
        self.set_outcome_description(
            "error", "Serialization failed because the input could not be processed."
        )
        self.add_input_key(
            "input",
            "ROS interface instance to serialize. The concrete type is selected "
            "through the interface_type parameter.",
        )
        self.add_output_key(
            "output",
            "Serialized byte array produced from the ROS interface instance.",
        )
        self.declare_parameter(
            "interface_type",
            "ROS interface type string such as geometry_msgs/msg/Pose or "
            "std_srvs/srv/Trigger_Request.",
            "geometry_msgs/msg/Pose",
        )

    def configure(self) -> None:
        self._interface_type = self.get_parameter("interface_type")
        self._interface_cls = resolve_interface_type(self._interface_type)

    def execute(self, blackboard: Blackboard) -> str:
        try:
            interface_value = blackboard["input"]
            if not isinstance(interface_value, self._interface_cls):
                yasmin.YASMIN_LOG_WARN(
                    f"RosSerializePyState expected '{self._interface_type}' but received '{type(interface_value).__name__}'"
                )
                return "type_error"

            blackboard["output"] = serialize_message(interface_value)
            return "succeed"
        except Exception as exc:
            yasmin.YASMIN_LOG_WARN(
                f"RosSerializePyState failed for '{self._interface_type}': {exc}"
            )
            return "error"
