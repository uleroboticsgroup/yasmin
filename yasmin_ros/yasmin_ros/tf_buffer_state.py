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

from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_ros.yasmin_node import YasminNode

import yasmin
from yasmin import Blackboard, State


class TfBufferState(State):
    """
    State that creates a tf2 buffer and transform listener.

    The created objects are stored in the blackboard under the keys
    ``tf_buffer`` and ``tf_listener`` so that following states can reuse them
    for transform lookups.
    """

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])
        self._node = None
        self._cache_time_sec = 10.0

        self.set_description(
            "Creates a shared tf2 buffer and transform listener and writes "
            "them to blackboard keys 'tf_buffer' and 'tf_listener'. Following "
            "states can reuse these objects to perform transform lookups."
        )
        self.set_outcome_description(
            SUCCEED,
            "The tf2 buffer and transform listener were created successfully.",
        )
        self.set_outcome_description(
            ABORT,
            "Failed to create the tf2 buffer or transform listener.",
        )
        self.add_output_key(
            "tf_buffer", "Created tf2_ros.Buffer instance used for lookups."
        )
        self.add_output_key(
            "tf_listener",
            "Created tf2_ros.TransformListener instance that keeps the buffer updated.",
        )
        self.declare_parameter(
            "cache_time_sec",
            "How many seconds of transform history the tf2 buffer should keep.",
            10.0,
        )

    def configure(self) -> None:
        self._cache_time_sec = float(self.get_parameter("cache_time_sec"))

        if self._node is None:
            self._node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard) -> str:
        try:
            tf_buffer = Buffer(cache_time=Duration(seconds=self._cache_time_sec))
            tf_listener = TransformListener(tf_buffer, self._node)
            blackboard["tf_buffer"] = tf_buffer
            blackboard["tf_listener"] = tf_listener
            return SUCCEED
        except Exception as exc:
            yasmin.YASMIN_LOG_ERROR(f"TfBufferState failed to create tf2 objects: {exc}")
            return ABORT
