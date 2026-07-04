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

import unittest

import rclpy
from geometry_msgs.msg import Pose
from rclpy.serialization import serialize_message

from yasmin import Blackboard
from yasmin_ros import RosDeserializePyState
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.ros_clients_cache import ROSClientsCache


class TestRosDeserializePyState(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        ROSClientsCache.clear_all()
        YasminNode.destroy_instance()
        rclpy.shutdown()

    def test_configure_invalid_interface_raises(self):
        state = RosDeserializePyState()
        state.set_parameter("interface_type", "does_not_exist")

        with self.assertRaises(Exception):
            state.configure()

    def test_execute_succeeds_for_pose(self):
        state = RosDeserializePyState()
        state.configure()

        pose = Pose()
        pose.position.x = 4.0
        pose.position.y = 5.0
        pose.position.z = 6.0
        pose.orientation.z = 0.25
        pose.orientation.w = 0.75

        blackboard = Blackboard()
        blackboard["input"] = serialize_message(pose)

        self.assertEqual("succeed", state(blackboard))
        output_pose = blackboard["output"]
        self.assertIsInstance(output_pose, Pose)
        self.assertEqual(output_pose.position.x, pose.position.x)
        self.assertEqual(output_pose.position.y, pose.position.y)
        self.assertEqual(output_pose.position.z, pose.position.z)
        self.assertEqual(output_pose.orientation.z, pose.orientation.z)
        self.assertEqual(output_pose.orientation.w, pose.orientation.w)

    def test_execute_returns_type_error_for_non_bytes_input(self):
        state = RosDeserializePyState()
        state.configure()

        blackboard = Blackboard()
        blackboard["input"] = "not bytes"

        self.assertEqual("type_error", state(blackboard))


if __name__ == "__main__":
    unittest.main()
