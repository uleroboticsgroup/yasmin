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

from yasmin import set_py_loggers
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from example_interfaces.action import Fibonacci

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


class TestActionClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_py_loggers()

        cls.aux_node = AuxNode()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.aux_node)

        cls.spin_thread = Thread(target=cls.executor.spin)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_action_client_succeed(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        state = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(SUCCEED, state())

    def test_action_client_cache(self):
        ROSClientsCache.clear_all()
        self.assertEqual(0, ROSClientsCache.get_action_clients_count())

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        state1 = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(1, ROSClientsCache.get_action_clients_count())

        state2 = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(1, ROSClientsCache.get_action_clients_count())

        state3 = ActionState(Fibonacci, "test2", create_goal_cb)
        self.assertEqual(2, ROSClientsCache.get_action_clients_count())

    def test_action_client_result_handler(self):

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

    def test_action_client_cancel(self):

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

    def test_action_client_abort(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = -1
            return goal

        state = ActionState(Fibonacci, "test", create_goal_cb)
        self.assertEqual(ABORT, state())

    def test_action_client_retry_wait_timeout(self):
        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 3
            return goal

        retries = 3

        ## Capture the logs
        with self.assertLogs("root", level="WARNING") as captured:
            state = ActionState(
                Fibonacci,
                "test1",
                create_goal_cb,
                wait_timeout=0.1,
                maximum_retry=retries,
            )
            self.assertEqual(TIMEOUT, state())

        ## Check that the number of WARNING logs is correct
        self.assertEqual(
            (retries * 2) + 1,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )

    def test_action_client_retry_response_timeout(self):
        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 3
            return goal

        retries = 3

        ## Capture the logs
        with self.assertLogs("root", level="WARNING") as captured:
            state = ActionState(
                Fibonacci,
                "test",
                create_goal_cb,
                response_timeout=0.1,
                maximum_retry=retries,
            )
            self.assertEqual(TIMEOUT, state())

        ## Check that the number of WARNING logs is correct
        self.assertEqual(
            (retries * 2) + 1,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )


if __name__ == "__main__":
    unittest.main()
