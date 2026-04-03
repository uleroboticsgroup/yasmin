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
from rclpy.serialization import serialize_message

from yasmin import Blackboard
from yasmin_ros import RosDeserializePyState


class TestRosDeserializePyState(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
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
