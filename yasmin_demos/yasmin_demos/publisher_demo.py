#!/usr/bin/env python3

# Copyright (C) 2025 Miguel Ángel González Santamarta
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

import time
import rclpy
from std_msgs.msg import Int32

import yasmin
from yasmin.cb_state import CbState
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard

from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros import PublisherState
from yasmin_ros.ros_logs import set_ros_loggers

from yasmin_viewer.yasmin_viewer_pub import YasminViewerPub


class PublishIntState(PublisherState):
    """
    PublishIntState is a YASMIN ROS publisher state that sends incrementing integers
    to the 'count' topic using std_msgs.msg.Int32 messages.

    This state increments a counter on the blackboard and publishes it.
    """

    def __init__(self):
        """
        Initializes the PublishIntState with the topic 'count' and a message creation callback.
        """
        super().__init__("count", self.create_int_msg)

    def create_int_msg(self, blackboard: Blackboard) -> Int32:
        """
        Generates a std_msgs.msg.Int32 message with an incremented counter value.

        Parameters:
            blackboard (Blackboard): The shared data store between states.

        Returns:
            Int32: A ROS message containing the updated counter.
        """
        # Get and increment the counter from the blackboard
        counter = blackboard.get("counter", 0)
        counter += 1
        blackboard.set("counter", counter)

        # Log the message creation
        yasmin.YASMIN_LOG_INFO(f"Creating message {counter}")

        # Create and return the message
        msg = Int32()
        msg.data = counter
        return msg


def check_count(blackboard: Blackboard) -> str:
    """
    Checks the current counter against a max threshold to determine state transition.

    Parameters:
        blackboard (Blackboard): The shared data store between states.

    Returns:
        str: The outcome string ('outcome1' or 'outcome2').
    """
    # Simulate processing time
    time.sleep(1)

    # Retrieve the counter and max value from blackboard
    count = blackboard.get("counter", 0)
    max_count = blackboard.get("max_count", 10)

    yasmin.YASMIN_LOG_INFO(f"Checking count: {count}")

    # Determine and return the outcome based on the counter value
    if count >= max_count:
        return "outcome1"
    else:
        return "outcome2"


def main(args=None):
    """
    Main function to initialize ROS 2, configure logging, build the YASMIN state machine,
    and execute it until the max_count is reached.

    Args:
        args (list, optional): Command-line arguments passed to rclpy.init().
    """
    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")
    rclpy.init(args=args)

    # Configure YASMIN to use ROS-based logging
    set_ros_loggers()

    # Create the state machine with 'SUCCEED' as the terminal outcome
    sm = StateMachine([SUCCEED])

    # Ensure the state machine cancels on shutdown
    def on_shutdown():
        if sm.is_running():
            sm.cancel_state()

    rclpy.get_default_context().on_shutdown(on_shutdown)

    # Add the publishing state which loops until the condition is met
    sm.add_state(
        "PUBLISHING_INT",
        PublishIntState(),
        {
            SUCCEED: "CHECKING_COUNTS",
        },
    )

    # Add the conditional check state
    sm.add_state(
        "CHECKING_COUNTS",
        CbState(["outcome1", "outcome2"], check_count),
        {
            "outcome1": SUCCEED,
            "outcome2": "PUBLISHING_INT",
        },
    )

    # Launch YASMIN Viewer publisher for state visualization
    YasminViewerPub("YASMIN_PUBLISHER_DEMO", sm)

    # Initialize blackboard with counter values
    blackboard = Blackboard()
    blackboard.set("counter", 0)
    blackboard.set("max_count", 10)

    # Run the state machine and log the outcome
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_INFO(str(e))

    # Shutdown ROS
    rclpy.shutdown()


if __name__ == "__main__":
    main()
