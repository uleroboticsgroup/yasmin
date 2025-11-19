# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import unittest

from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros import PublisherState
from yasmin_ros.basic_outcomes import SUCCEED

from std_msgs.msg import String

import rclpy


class TestYasminRos(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_publisher(self):

        def create_msg_handler(blackboard):
            msg = String()
            msg.data = "data"
            return msg

        state = PublisherState(String, "test", create_msg_handler)
        self.assertEqual(SUCCEED, state())

    def test_publisher_cache(self):
        ROSClientsCache.clear_all()
        self.assertEqual(0, ROSClientsCache.get_publishers_count())

        def create_msg_handler(blackboard):
            msg = String()
            msg.data = "data"
            return msg

        state1 = PublisherState(String, "test", create_msg_handler)
        self.assertEqual(1, ROSClientsCache.get_publishers_count())

        state2 = PublisherState(String, "test", create_msg_handler)
        self.assertEqual(1, ROSClientsCache.get_publishers_count())

        state3 = PublisherState(String, "test2", create_msg_handler)
        self.assertEqual(2, ROSClientsCache.get_publishers_count())


if __name__ == "__main__":
    unittest.main()
