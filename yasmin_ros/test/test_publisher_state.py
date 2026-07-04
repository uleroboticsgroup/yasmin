# Copyright (C) 2023 Miguel Ángel González Santamarta
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

from yasmin_ros import PublisherState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros.yasmin_node import YasminNode

import rclpy
from std_msgs.msg import String


class TestYasminRos(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        ROSClientsCache.clear_all()
        YasminNode.destroy_instance()
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
