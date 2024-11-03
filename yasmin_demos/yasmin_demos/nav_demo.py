#!/usr/bin/env python3

# Copyright (C) 2024  Miguel Ángel González Santamarta
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

import random
import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

# Constants for state outcomes
HAS_NEXT = "has_next"  ##< Indicates there are more waypoints
END = "end"  ##< Indicates no more waypoints


class Nav2State(ActionState):
    """
    ActionState for navigating to a specified pose using ROS 2 Navigation.

    Attributes:
        None

    Methods:
        create_goal_handler(blackboard: Blackboard) -> NavigateToPose.Goal:
            Creates the navigation goal from the blackboard.
    """

    def __init__(self) -> None:
        """
        Initializes the Nav2State.

        Calls the parent constructor to set up the action with:
        - Action type: NavigateToPose
        - Action name: /navigate_to_pose
        - Callback for goal creation: create_goal_handler
        - Outcomes: None, since it will use default outcomes (SUCCEED, ABORT, CANCEL)
        """
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        """
        Creates a goal for navigation based on the current pose in the blackboard.

        Args:
            blackboard (Blackboard): The blackboard instance holding current state data.

        Returns:
            NavigateToPose.Goal: The constructed goal for the navigation action.
        """
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"  # Set the reference frame to 'map'
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    """
    Initializes waypoints in the blackboard for navigation.

    Args:
        blackboard (Blackboard): The blackboard instance to store waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["waypoints"] = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED


def take_random_waypoint(blackboard: Blackboard) -> str:
    """
    Selects a random set of waypoints from the available waypoints.

    Args:
        blackboard (Blackboard): The blackboard instance to store random waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), blackboard["waypoints_num"]
    )
    return SUCCEED


def get_next_waypoint(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """
    if not blackboard["random_waypoints"]:
        return END

    wp_name = blackboard["random_waypoints"].pop(0)  # Get the next waypoint name
    wp = blackboard["waypoints"][wp_name]  # Get the waypoint coordinates

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]
    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard["pose"] = pose  # Update blackboard with new pose
    blackboard["text"] = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


# main function
def main() -> None:
    """
    Initializes the ROS 2 node, sets up state machines for navigation, and executes the FSM.

    Handles cleanup and shutdown of the ROS 2 node upon completion.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_nav2_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers for debugging
    set_ros_loggers()

    # Create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # Add states to the state machine
    sm.add_state(
        "CREATING_WAYPOINTS",
        CbState([SUCCEED], create_waypoints),
        transitions={
            SUCCEED: "TAKING_RANDOM_WAYPOINTS",
        },
    )
    sm.add_state(
        "TAKING_RANDOM_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={
            SUCCEED: "NAVIGATING",
        },
    )

    nav_sm.add_state(
        "GETTING_NEXT_WAYPOINT",
        CbState([END, HAS_NEXT], get_next_waypoint),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "NAVIGATING",
        },
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={
            SUCCEED: "GETTING_NEXT_WAYPOINT",
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )

    sm.add_state(
        "NAVIGATING",
        nav_sm,
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub("YASMIN_NAV_DEMO", sm)

    # Execute the state machine
    blackboard = Blackboard()
    blackboard["waypoints_num"] = 2  # Set the number of waypoints to navigate

    try:
        outcome = sm(blackboard)  # Run the state machine with the blackboard
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()  # Handle manual interruption

    # Shutdown ROS 2
    if rclpy.ok():
        if sm.is_running():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
