#!/usr/bin/env python3

# Copyright (C) 2026 Miguel Ángel González Santamarta
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

import time

import rclpy

import yasmin
from yasmin import Blackboard, JoinState, OrthogonalState, State, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub


class WorkerState(State):
    def __init__(self, name: str, max_count: int, sleep_ms: int = 300) -> None:
        super().__init__(["working", "done"])
        self.name = name
        self.counter = 0
        self.max_count = max_count
        self.sleep_ms = sleep_ms

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO(
            f"WorkerState [{self.name}]: iteration {self.counter + 1}/{self.max_count}"
        )
        time.sleep(self.sleep_ms / 1000.0)

        self.counter += 1
        if self.counter >= self.max_count:
            return "done"
        return "working"


def main() -> None:
    rclpy.init()
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_orthogonal_sync_demo")

    SYNC_ID = "mid_sync"

    # Region A: 3 iterations -> JoinState -> 2 more iterations
    reg_a = StateMachine(outcomes=["done"])
    reg_a.set_name("Region A")
    reg_a.add_state(
        "WORK_1",
        WorkerState("A.1", 3, 200),
        transitions={"working": "WORK_1", "done": "SYNC"},
    )
    reg_a.add_state(
        "SYNC",
        JoinState(SYNC_ID),
        transitions={"joined": "WORK_2"},
    )
    reg_a.add_state(
        "WORK_2",
        WorkerState("A.2", 2, 200),
        transitions={"working": "WORK_2", "done": "done"},
    )
    reg_a.set_start_state("WORK_1")

    # Region B: 2 iterations -> JoinState -> 3 more iterations
    reg_b = StateMachine(outcomes=["done"])
    reg_b.set_name("Region B")
    reg_b.add_state(
        "WORK_1",
        WorkerState("B.1", 2, 200),
        transitions={"working": "WORK_1", "done": "SYNC"},
    )
    reg_b.add_state(
        "SYNC",
        JoinState(SYNC_ID),
        transitions={"joined": "WORK_2"},
    )
    reg_b.add_state(
        "WORK_2",
        WorkerState("B.2", 3, 200),
        transitions={"working": "WORK_2", "done": "done"},
    )
    reg_b.set_start_state("WORK_1")

    # Orthogonal state runs both regions in parallel with sync point
    ort = OrthogonalState(
        default_outcome="timeout",
        outcome_map={"success": {"A": "done", "B": "done"}},
    )
    ort.set_description("Runs two regions that sync at a JoinState before continuing.")
    ort.add_region("A", reg_a)
    ort.add_region("B", reg_b)

    # Root state machine
    sm = StateMachine(outcomes=["success", "timeout"], handle_sigint=True)
    sm.set_description("Demonstrates orthogonal state with JoinState synchronization.")
    sm.add_state(
        "PARALLEL",
        ort,
        transitions={"success": "success", "timeout": "timeout"},
    )
    sm.set_start_state("PARALLEL")

    YasminViewerPub(sm, "YASMIN_ORTHOGONAL_SYNC_DEMO")

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    YasminNode.destroy_instance()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
