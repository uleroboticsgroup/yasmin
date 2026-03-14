#!/usr/bin/env python3

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


from geometry_msgs.msg import Pose
from rclpy.serialization import deserialize_message

import yasmin
from yasmin import Blackboard, State


class PoseReaderState(State):
    """
    Represents a state that reads and deserializes a Pose message from the blackboard.
    """

    def __init__(self) -> None:
        """
        Initializes the PoseReaderState instance, setting up the outcome.

        Outcomes:
            outcome2: Indicates the serialized Pose was read successfully.
        """
        super().__init__(outcomes=["outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the PoseReaderState.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome2".

        Raises:
            Exception: May raise exceptions related to blackboard access or deserialization.
        """
        yasmin.YASMIN_LOG_INFO("Executing state POSE_READER")

        pose_type = blackboard["pose_bytes__type"]
        pose_bytes = blackboard["pose_bytes"]
        pose = deserialize_message(pose_bytes, Pose)

        yasmin.YASMIN_LOG_INFO(f"Stored type: {pose_type}")
        yasmin.YASMIN_LOG_INFO(
            f"Position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}"
        )
        yasmin.YASMIN_LOG_INFO(
            f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}"
        )

        return "outcome2"
