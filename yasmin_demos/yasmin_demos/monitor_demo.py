#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta
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

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import yasmin
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    """
    MonitorState subclass to handle Odometry messages.

    This state monitors Odometry messages from the specified ROS topic,
    logging them and transitioning based on the number of messages received.

    Attributes:
        times (int): The number of messages to monitor before transitioning
                     to the next outcome.

    Parameters:
        times (int): The initial count of how many Odometry messages to
                     process before changing state.

    Methods:
        monitor_handler(blackboard: Blackboard, msg: Odometry) -> str:
            Handles incoming Odometry messages, logging the message and
            returning the appropriate outcome based on the remaining count.
    """

    def __init__(self, times: int) -> None:
        """
        Initializes the PrintOdometryState.

        Parameters:
            times (int): The number of Odometry messages to monitor before
                         transitioning to the next outcome.
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
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        """
        Handles incoming Odometry messages.

        This method is called whenever a new Odometry message is received.
        It logs the message, decrements the count of messages to process,
        and determines the next state outcome.

        Parameters:
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


def main():
    """
    Main function to initialize and run the ROS 2 state machine.

    This function initializes ROS 2, sets up logging, creates a finite state
    machine (FSM), adds states to the FSM, and executes the FSM. It handles
    cleanup and shutdown of ROS 2 gracefully.

    Exceptions:
        KeyboardInterrupt: Caught to allow graceful cancellation of the
                          state machine during execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            TIMEOUT: "outcome4",
            CANCEL: "outcome4",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # Execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
