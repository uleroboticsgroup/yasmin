#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta
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
import json
from threading import Thread
from typing import List, Dict

from flask import Flask
from waitress import serve
from expiringdict import ExpiringDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import ament_index_python

from yasmin_msgs.msg import StateMachine, State, Transition


class YasminFsmViewerNode(Node):
    """
    A ROS 2 node that serves as a viewer for the finite state machines (FSMs)
    using a Flask web server.

    This class subscribes to FSM updates and serves them over HTTP. It utilizes
    a dictionary with expiring entries to manage the FSM data.

    Attributes:
        __fsm_dict (ExpiringDict): A dictionary that stores FSM data with
                                    automatic expiration.

    Methods:
        start_backend_server(): Initializes and starts the Flask server.
        start_subscriber(): Subscribes to the FSM state updates.
        transition_msg_to_dict(tansitions): Converts a list of transitions to a dictionary.
        state_msg_to_dict(msg): Converts a State message to a dictionary.
        msg_to_dict(msg): Converts a StateMachine message to a list of dictionaries.
        fsm_viewer_cb(msg): Callback function for processing FSM updates.
    """

    def __init__(self) -> None:
        """
        Initializes the YasminFsmViewerNode node.

        Declares parameters for the host and port, sets up the expiring dictionary,
        starts the subscriber thread, and initializes the backend server.
        """
        super().__init__("yasmin_viewer")

        ## A dictionary that stores FSM data with automatic expiration.
        self.__fsm_dict: ExpiringDict = ExpiringDict(max_len=300, max_age_seconds=3)

        # Declare parameters for the Flask server
        self.declare_parameters(
            namespace="",
            parameters=[
                ("host", "localhost"),
                ("port", 5000),
            ],
        )

        # Start the subscriber in a separate thread
        thread_subscriber = Thread(target=self.start_subscriber)
        thread_subscriber.start()

        # Start the backend server to serve FSM data
        self.start_backend_server()

    def start_backend_server(self) -> None:
        """
        Initializes and starts the Flask backend server.

        Sets up the routes for serving static files and retrieving FSM data.

        :raises Exception: If the server fails to start.
        """
        app = Flask(
            "yasmin_viewer",
            static_folder=ament_index_python.get_package_share_directory("yasmin_viewer")
            + "/yasmin_viewer_web_client",
            static_url_path="/",
        )

        @app.route("/")
        def index():
            """Serves the index.html file."""
            return app.send_static_file("index.html")

        @app.route("/get_fsms", methods=["GET"])
        def get_fsms():
            """Returns the current FSMs as JSON."""
            return json.dumps(self.__fsm_dict)

        @app.route("/get_fsm/<fsm_name>", methods=["GET"])
        def get_fsm(fsm_name):
            """
            Returns the specified FSM's data as JSON.

            :param fsm_name: The name of the FSM to retrieve.
            :return: JSON representation of the FSM data or an empty JSON object.
            """
            if fsm_name in self.__fsm_dict:
                return json.dumps(self.__fsm_dict[fsm_name])

            return json.dumps({})

        _host = str(self.get_parameter("host").value)
        _port = int(self.get_parameter("port").value)

        self.get_logger().info(f"Started Yasmin viewer on http://{_host}:{str(_port)}")
        serve(app, host=_host, port=_port)

    def start_subscriber(self) -> None:
        """
        Subscribes to the FSM state updates and starts the ROS spinning.

        This method creates a subscription to the '/fsm_viewer' topic and
        processes incoming messages.

        :raises ExternalShutdownException: If the ROS 2 node is externally shutdown.
        """
        self.create_subscription(StateMachine, "/fsm_viewer", self.fsm_viewer_cb, 10)

        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def transition_msg_to_dict(self, transitions: List[Transition]) -> Dict:
        """
        Converts a list of Transition messages to a dictionary.

        :param transitions: A list of Transition messages to convert.
        :return: A dictionary mapping outcomes to states.
        """
        transition_dict = {}
        for transition in transitions:
            transition_dict[transition.outcome] = transition.state
        return transition_dict

    def state_msg_to_dict(self, msg: State) -> Dict:
        """
        Converts a State message to a dictionary representation.

        :param msg: The State message to convert.
        :return: A dictionary containing state attributes.
        """
        state_dict = {
            "id": msg.id,
            "parent": msg.parent,
            "name": msg.name,
            "transitions": self.transition_msg_to_dict(msg.transitions),
            "outcomes": msg.outcomes,
            "is_fsm": msg.is_fsm,
            "current_state": msg.current_state,
        }
        return state_dict

    def msg_to_dict(self, msg: StateMachine) -> Dict:
        """
        Converts a StateMachine message to a list of dictionaries.

        :param msg: The StateMachine message to convert.
        :return: A list of dictionaries representing the states in the StateMachine.
        """
        states_dict = []
        for state in msg.states:
            states_dict.append(self.state_msg_to_dict(state))
        return states_dict

    def fsm_viewer_cb(self, msg: StateMachine) -> None:
        """
        Callback function for processing incoming StateMachine messages.

        Waits for the server to start before storing the received FSM data.

        :param msg: The StateMachine message containing the FSM data.
        """

        if msg.states:
            self.__fsm_dict[msg.states[0].name] = self.msg_to_dict(msg)


def main() -> None:
    """
    Main entry point for the YasminFsmViewerNode node.

    Initializes the ROS 2 communication and creates an instance of
    YasminFsmViewerNode.
    """
    rclpy.init()
    YasminFsmViewerNode()


if __name__ == "__main__":
    main()
