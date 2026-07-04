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

from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_ros.ros_state_utils import resolve_node

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
        self._prev_tf_buffer = None
        self._prev_tf_listener = None

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
            self._node = resolve_node()

    def execute(self, blackboard: Blackboard) -> str:
        try:
            # Clean up previous instances to avoid resource leaks
            if self._prev_tf_listener is not None:
                self._prev_tf_listener.__del__()
            if self._prev_tf_buffer is not None:
                self._prev_tf_buffer.__del__()

            tf_buffer = Buffer(cache_time=Duration(seconds=self._cache_time_sec))
            tf_listener = TransformListener(tf_buffer, self._node)
            self._prev_tf_buffer = tf_buffer
            self._prev_tf_listener = tf_listener
            blackboard["tf_buffer"] = tf_buffer
            blackboard["tf_listener"] = tf_listener
            return SUCCEED
        except Exception as exc:
            yasmin.YASMIN_LOG_ERROR(f"TfBufferState failed to create tf2 objects: {exc}")
            return ABORT
