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
from std_msgs.msg import String
from yasmin import Blackboard
from yasmin_ros import RosSerializePyState
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.ros_clients_cache import ROSClientsCache


class TestRosSerializePyState(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        ROSClientsCache.clear_all()
        YasminNode.destroy_instance()
        rclpy.shutdown()

    def test_configure_invalid_interface_raises(self):
        state = RosSerializePyState()
        state.set_parameter("interface_type", "does_not_exist")

        with self.assertRaises(Exception):
            state.configure()

    def test_execute_succeeds_for_pose(self):
        state = RosSerializePyState()
        state.configure()

        pose = Pose()
        pose.position.x = 1.25
        pose.position.y = -2.5
        pose.position.z = 3.75
        pose.orientation.w = 0.5

        blackboard = Blackboard()
        blackboard["input"] = pose

        self.assertEqual("succeed", state(blackboard))
        self.assertIsInstance(blackboard["output"], (bytes, bytearray))
        self.assertGreater(len(blackboard["output"]), 0)

    def test_execute_returns_type_error_for_wrong_input_type(self):
        state = RosSerializePyState()
        state.configure()

        msg = String()
        msg.data = "wrong type"

        blackboard = Blackboard()
        blackboard["input"] = msg

        self.assertEqual("type_error", state(blackboard))


if __name__ == "__main__":
    unittest.main()
