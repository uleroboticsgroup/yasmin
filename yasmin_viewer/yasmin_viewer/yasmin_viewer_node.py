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

import json
from threading import Thread
from typing import List, Dict

from flask import Flask
from flask_socketio import SocketIO
from waitress import serve
from expiringdict import ExpiringDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import ament_index_python

from yasmin_msgs.msg import StateMachine, State, Transition


class YasminViewerNode(Node):
    """
    A ROS 2 node that serves as a viewer for the finite state machines (FSMs)
    using a Flask web server.

    This class subscribes to FSM updates and serves them over HTTP. It utilizes
    a dictionary with expiring entries to manage the FSM data.
    """

    def __init__(self) -> None:
        """
        Initializes the YasminViewerNode node.

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
                ("msg_per_second", 30),
            ],
        )

        # Start the subscriber in a separate thread
        self.emit_timer = None
        thread_subscriber = Thread(target=self.start_ros)
        thread_subscriber.start()

        # Start the backend server to serve FSM data
        self.socketio: SocketIO = None
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

        # Initialize SocketIO
        self.socketio = SocketIO(app, cors_allowed_origins="*")

        @app.route("/")
        def index():
            """Serves the index.html file."""
            return app.send_static_file("index.html")

        @self.socketio.on("connect")
        def handle_connect():
            self.get_logger().info("Client connected")

        @self.socketio.on("disconnect")
        def handle_disconnect():
            self.get_logger().info("Client disconnected")

        _host = str(self.get_parameter("host").value)
        _port = int(self.get_parameter("port").value)

        self.get_logger().info(f"Started Yasmin viewer on http://{_host}:{str(_port)}")
        serve(app, host=_host, port=_port)

    def start_ros(self) -> None:
        """
        Subscribes to the FSM state updates and starts the ROS spinning.

        This method creates a subscription to the '/fsm_viewer' topic and
        processes incoming messages.

        :raises ExternalShutdownException: If the ROS 2 node is externally shutdown.
        """
        self.create_subscription(StateMachine, "/fsm_viewer", self.fsm_viewer_cb, 10)
        self.emit_timer = self.create_timer(
            1 / self.get_parameter("msg_per_second").value, self.emit_fsm_data
        )

        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def transition_msg_to_list(self, transitions: List[Transition]) -> List:
        """
        Converts a list of Transition messages to a list.

        :param transitions: A list of Transition messages to convert.
        :return: A list mapping outcomes to states.
        """
        return [[t.outcome, t.state] for t in transitions]

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
            "transitions": self.transition_msg_to_list(msg.transitions),
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

    def emit_fsm_data(self) -> None:
        """
        Emits the current FSM data to connected clients via socket.io.
        """
        if self.socketio:
            self.socketio.emit("fsms_update", json.dumps(self.__fsm_dict))


def main() -> None:
    rclpy.init()
    node = YasminViewerNode()
    node.destroy_node()


if __name__ == "__main__":
    main()
