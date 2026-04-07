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

import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from yasmin import Blackboard, State
from yasmin_ros import TfBufferState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED


class LookupTfState(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:
        tf_buffer = blackboard["tf_buffer"]
        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            try:
                transform = tf_buffer.lookup_transform("map", "base_link", Time())
                if abs(transform.transform.translation.x - 1.23) < 1e-6:
                    return SUCCEED
            except Exception:
                pass
            time.sleep(0.05)

        return ABORT


class TestTransformBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_buffer_py_test")
        self._broadcaster = TransformBroadcaster(self)

    def publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 1.23
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(transform)


class TestTfBufferState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_creates_buffer_and_listener_in_blackboard(self):
        blackboard = Blackboard()
        state = TfBufferState()
        state.configure()

        self.assertEqual(SUCCEED, state(blackboard))
        self.assertIn("tf_buffer", blackboard)
        self.assertIn("tf_listener", blackboard)

    def test_lookup_from_following_state(self):
        broadcaster_node = TestTransformBroadcasterNode()
        running = True

        def publish_loop() -> None:
            while running:
                broadcaster_node.publish_transform()
                time.sleep(0.05)

        publisher_thread = threading.Thread(target=publish_loop)
        publisher_thread.start()

        try:
            blackboard = Blackboard()
            create_buffer_state = TfBufferState()
            create_buffer_state.configure()
            lookup_state = LookupTfState()

            self.assertEqual(SUCCEED, create_buffer_state(blackboard))
            self.assertEqual(SUCCEED, lookup_state(blackboard))
        finally:
            running = False
            publisher_thread.join()
            broadcaster_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
