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
        self.set_description(
            "Reads a serialized Pose message from the blackboard, deserializes it, and logs the position and orientation."
        )
        self.add_input_key(
            "pose_bytes",
            "Serialized Pose message stored as bytes in the blackboard.",
        )
        self.add_input_key(
            "pose_bytes__type",
            "Type information for the serialized Pose message.",
            "geometry_msgs/msg/Pose",
        )

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
