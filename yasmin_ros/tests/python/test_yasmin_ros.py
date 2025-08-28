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


import time
import unittest
from threading import Thread

from yasmin_ros import ActionState, ServiceState, MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import CancelResponse, GoalResponse


class AuxNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.action_server = ActionServer(
            self,
            Fibonacci,
            "test",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_action,
            cancel_callback=self.cancel_action,
            callback_group=ReentrantCallbackGroup(),
        )

        self.service_server = self.create_service(
            AddTwoInts, "test", self.execute_service
        )

        self.pub = self.create_publisher(String, "test", 10)
        self.timer = self.create_timer(1, self.publis_msgs)

    def goal_callback(self, goal_request) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        goal_handle.execute()

    def execute_action(self, goal_handle):
        result = Fibonacci.Result()

        request = goal_handle.request

        if request.order < 0:
            goal_handle.abort()

        else:
            time.sleep(3)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.succeed()

        return result

    def cancel_action(self, goal_handle) -> int:
        return CancelResponse.ACCEPT

    def execute_service(self, request, response):
        response.sum = request.a + request.b
        return response

    def publis_msgs(self) -> None:
        msg = String()
        msg.data = "data"
        self.pub.publish(msg)


class TestYasminRos(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.aux_node = AuxNode()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.aux_node)

        cls.spin_thread = Thread(target=cls.executor.spin)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_action_succeed(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        state = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(SUCCEED, state())

    def test_action_result_handler(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def result_handler(blackboard, result):
            return "new_outcome"

        state = ActionState(
            Fibonacci, "test", create_goal_cb, ["new_outcome"], result_handler
        )
        self.assertEqual("new_outcome", state())

    def test_action_cancel(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 20
            return goal

        def cancel_state(state, seconds):
            time.sleep(seconds)
            state.cancel_state()

        state = ActionState(Fibonacci, "test", create_goal_cb)
        thread = Thread(
            target=cancel_state,
            args=(
                state,
                1,
            ),
        )
        thread.start()
        self.assertEqual(CANCEL, state())
        thread.join()

    def test_action_abort(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = -1
            return goal

        state = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(ABORT, state())

    def test_service(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        state = ServiceState(AddTwoInts, "test", create_request_cb)
        self.assertEqual(SUCCEED, state())

    def test_service_response_handler(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        def response_handler(blackboard, response):
            return "new_outcome"

        state = ServiceState(
            AddTwoInts, "test", create_request_cb, ["new_outcome"], response_handler
        )
        self.assertEqual("new_outcome", state())

    def test_monitor_timeout(self):

        def monitor_handler(blackboard, msg):
            return SUCCEED

        state = MonitorState(
            String, "test1", [SUCCEED], monitor_handler=monitor_handler, timeout=2
        )
        self.assertEqual(TIMEOUT, state())
