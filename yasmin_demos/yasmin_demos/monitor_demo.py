#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

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


import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import TIMEOUT
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    def __init__(self, times: int) -> None:
        super().__init__(
            Odometry,  # msg type
            "odom",  # topic name
            ["outcome1", "outcome2"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic sbscription
            msg_queue=10,  # queue of the monitor handler callback
            timeout=10,  # timeout to wait for msgs in seconds
            # if not None, CANCEL outcome is added
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        print(msg)

        self.times -= 1

        if self.times <= 0:
            return "outcome2"

        return "outcome1"


# main
def main():

    print("yasmin_monitor_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            TIMEOUT: "outcome4",
        },
    )

    # pub FSM info
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
