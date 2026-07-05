#!/usr/bin/env python3

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

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from yasmin_ros.basic_outcomes import CANCEL, TIMEOUT

import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import MonitorState, set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    """
    MonitorState subclass to handle Odometry messages.

    This state monitors Odometry messages from the specified ROS topic,
    logging them and transitioning based on the number of messages received.
    """

    def __init__(self) -> None:
        """
        Initializes the PrintOdometryState.
        """
        super().__init__(
            Odometry,  # msg type
            "odom",  # topic name
            ["outcome1", "outcome2"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic subscription
            msg_queue=10,  # queue for the monitor handler callback
            timeout=10,  # timeout to wait for messages in seconds
        )
        self.times = 5
        self.set_description(
            "Monitors Odometry messages from the 'odom' topic and logs them until a predefined number of messages has been received."
        )
        self.add_input_key(
            "odom",
            "Odometry message received from the monitored ROS topic.",
        )

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        """
        Handles incoming Odometry messages.

        This method is called whenever a new Odometry message is received.
        It logs the message, decrements the count of messages to process,
        and determines the next state outcome.

        Args:
            blackboard (Blackboard): The shared data storage for states.
            msg (Odometry): The incoming Odometry message.

        Returns:
            str: The next state outcome, either "outcome1" to continue
                 monitoring or "outcome2" to transition to the next state.

        Exceptions:
            None
        """
        yasmin.YASMIN_LOG_INFO(msg)

        self.times -= 1

        if self.times <= 0:
            return "outcome2"

        return "outcome1"


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"], handle_sigint=True)
    sm.set_description(
        "Continuously monitors the 'odom' topic and logs received Odometry messages until a fixed number of messages has been processed."
    )
    sm.add_input_key(
        "odom",
        "Odometry messages received from the monitored ROS topic.",
    )

    # Add states to the FSM
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            TIMEOUT: "outcome4",
            CANCEL: "outcome4",
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub(sm, "YASMIN_MONITOR_DEMO")

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    YasminNode.destroy_instance()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
