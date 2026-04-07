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
