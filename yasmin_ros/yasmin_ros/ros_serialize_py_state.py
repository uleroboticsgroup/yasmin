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
        self.set_outcome_description(
            "succeed", "Serialization finished successfully."
        )
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
