#!/usr/bin/env python3

# Copyright (C) 2024  Miguel Ángel González Santamarta

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


import random

import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

HAS_NEXT = "has_next"
END = "end"


class Nav2State(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None,  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:

        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    blackboard["waypoints"] = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED


def take_random_waypoint(blackboard) -> str:
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), blackboard["waypoints_num"]
    )
    return SUCCEED


def get_next_waypoint(blackboard: Blackboard) -> str:

    if not blackboard["random_waypoints"]:
        return END

    wp_name = blackboard["random_waypoints"].pop(0)
    wp = blackboard["waypoints"][wp_name]

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]

    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard["pose"] = pose
    blackboard["text"] = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


# main
def main():

    print("yasmin_nav2_demo")

    # init ROS 2
    rclpy.init()

    # create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # add states
    sm.add_state(
        "CREATING_WAYPOINTS",
        CbState([SUCCEED], create_waypoints),
        transitions={SUCCEED: "TAKING_RANDOM_WAYPOINTS"},
    )
    sm.add_state(
        "TAKING_RANDOM_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={SUCCEED: "NAVIGATING"},
    )

    nav_sm.add_state(
        "GETTING_NEXT_WAYPOINT",
        CbState([END, HAS_NEXT], get_next_waypoint),
        transitions={END: SUCCEED, HAS_NEXT: "NAVIGATING"},
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={SUCCEED: "GETTING_NEXT_WAYPOINT", CANCEL: CANCEL, ABORT: ABORT},
    )

    sm.add_state(
        "NAVIGATING",
        nav_sm,
        transitions={SUCCEED: SUCCEED, CANCEL: CANCEL, ABORT: ABORT},
    )

    # pub FSM info
    YasminViewerPub("YASMIN_NAV_DEMO", sm)

    # execute FSM
    blackboard = Blackboard()
    blackboard["waypoints_num"] = 2
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
