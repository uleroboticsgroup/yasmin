#!/usr/bin/env python3

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

import threading
import time
import rclpy

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from yasmin_factory import YasminFactory
from yasmin_msgs.action import RunStateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub
import yasmin


class FactoryActionServer:
    def __init__(self, node):
        self.node = node
        node.declare_parameter("state_machine_file", "")
        node.declare_parameter("enable_viewer_pub", True)
        self.default_file = node.get_parameter("state_machine_file").value
        self.enable_viewer = node.get_parameter("enable_viewer_pub").value
        self._lock = threading.Lock()
        self._busy = False
        self._cancel = False
        self._shutdown = False
        self._sm = None
        self._server = ActionServer(
            node,
            RunStateMachine,
            "run_state_machine",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal):
        with self._lock:
            filename = goal.state_machine_file or self.default_file
            if self._shutdown or self._busy or not filename:
                return GoalResponse.REJECT
            self._busy = True
            self._cancel = False
            return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        with self._lock:
            self._cancel = True
            sm = self._sm
        if sm is not None:
            sm.cancel_state_machine()
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle, state):
        msg = RunStateMachine.Feedback()
        msg.current_state = state
        goal_handle.publish_feedback(msg)

    def execute(self, goal_handle):
        result = RunStateMachine.Result()
        filename = goal_handle.request.state_machine_file or self.default_file
        viewer = None
        try:
            factory = YasminFactory()
            sm = factory.create_sm_from_file(filename)
            sm.set_sigint_handler(False)
            sm.add_start_cb(lambda _bb, state: self._feedback(goal_handle, state))
            sm.add_transition_cb(
                lambda _bb, _from, to, _out: self._feedback(goal_handle, to)
            )
            with self._lock:
                self._sm = sm
            viewer = YasminViewerPub(sm) if self.enable_viewer else None
            outcome = sm()
            result.outcome = (
                "" if self._cancel or goal_handle.is_cancel_requested else outcome
            )
            with self._lock:
                canceled = self._cancel or goal_handle.is_cancel_requested
            if canceled:
                goal_handle.canceled()
            else:
                goal_handle.succeed()
        except Exception as exc:
            result.outcome = ""
            with self._lock:
                canceled = self._cancel or goal_handle.is_cancel_requested
            if canceled:
                goal_handle.canceled()
            else:
                yasmin.YASMIN_LOG_ERROR(f"State machine execution failed: {exc}")
                goal_handle.abort()
        finally:
            if viewer is not None:
                viewer.shutdown()
            with self._lock:
                self._sm = None
                self._busy = False
        return result

    def shutdown(self):
        with self._lock:
            self._shutdown = True
            sm = self._sm
        if sm is not None:
            sm.cancel_state_machine()
        self._server.destroy()


def main():
    rclpy.init()
    set_ros_loggers()
    node = YasminNode.get_instance()
    server = FactoryActionServer(node)
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        YasminNode.destroy_instance()


if __name__ == "__main__":
    main()
