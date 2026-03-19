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

import asyncio
import json
from pathlib import Path
from threading import Lock, Thread
from typing import Dict, List, Optional

import ament_index_python
import rclpy
import socketio
import uvicorn
from expiringdict import ExpiringDict
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from yasmin_msgs.msg import State, StateMachine, Transition


class YasminViewerNode(Node):
    """
    A ROS 2 node that serves as a viewer for the finite state machines (FSMs)
    using a FastAPI web server.

    This class subscribes to FSM updates and serves them over HTTP. It utilizes
    a dictionary with expiring entries to manage the FSM data.
    """

    def __init__(self) -> None:
        """
        Initializes the YasminViewerNode node.

        Declares parameters for the host and port, sets up the expiring dictionary,
        and starts the ROS subscriber thread.
        """
        super().__init__("yasmin_viewer")

        ## A dictionary that stores FSM data with automatic expiration.
        self.__fsm_dict: ExpiringDict = ExpiringDict(max_len=300, max_age_seconds=3)
        self.__fsm_lock = Lock()
        self.__last_emitted_payload = ""
        self.__loop: Optional[asyncio.AbstractEventLoop] = None

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
        self._ros_thread = Thread(target=self.start_ros, daemon=True)
        self._ros_thread.start()

        self.socketio: Optional[socketio.AsyncServer] = None

    def create_app(self):
        """
        Creates the FastAPI application.

        :return: Configured FastAPI application.
        """
        client_dir = (
            Path(ament_index_python.get_package_share_directory("yasmin_viewer"))
            / "yasmin_viewer_web_client"
        ).resolve()
        index_file = client_dir / "index.html"

        app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

        @app.on_event("startup")
        async def startup_event() -> None:
            self.__loop = asyncio.get_running_loop()

        @app.get("/api/fsms")
        async def get_fsms() -> JSONResponse:
            return JSONResponse(content=json.loads(self.get_fsm_payload()))

        @app.get("/{full_path:path}")
        async def serve_client(full_path: str = "") -> FileResponse:
            if full_path:
                requested_path = (client_dir / full_path).resolve()
                if client_dir in requested_path.parents and requested_path.is_file():
                    return FileResponse(requested_path)

            return FileResponse(index_file)

        # Initialize SocketIO
        self.socketio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*",
        )

        @self.socketio.event
        async def connect(sid, environ):
            self.get_logger().info("Client connected")

        @self.socketio.event
        async def disconnect(sid):
            self.get_logger().info("Client disconnected")

        return socketio.ASGIApp(
            self.socketio,
            other_asgi_app=app,
            socketio_path="socket.io",
        )

    def start_backend_server(self) -> None:
        """
        Initializes and starts the FastAPI backend server.
        """
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        self.get_logger().info(f"Started Yasmin viewer on http://{host}:{port}")
        uvicorn.run(self.create_app(), host=host, port=port, log_level="warning")

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

    def get_fsm_payload(self) -> str:
        """
        Serializes the current FSM cache.

        :return: JSON string containing all active FSMs.
        """
        with self.__fsm_lock:
            snapshot = {}
            for key in list(self.__fsm_dict.keys()):
                try:
                    snapshot[key] = self.__fsm_dict[key]
                except KeyError:
                    pass

        return json.dumps(snapshot)

    def fsm_viewer_cb(self, msg: StateMachine) -> None:
        """
        Callback function for processing incoming StateMachine messages.

        :param msg: The StateMachine message containing the FSM data.
        """
        if msg.states:
            with self.__fsm_lock:
                self.__fsm_dict[msg.states[0].name] = self.msg_to_dict(msg)

    def _handle_emit_result(self, future) -> None:
        """
        Handles exceptions from cross-thread socket.io emits.

        :param future: Scheduled emit future.
        """
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Socket.IO emit failed: {exc}")

    def emit_fsm_data(self) -> None:
        """
        Emits the current FSM data to connected clients via socket.io.
        """
        payload = self.get_fsm_payload()
        if payload == self.__last_emitted_payload:
            return

        self.__last_emitted_payload = payload

        if self.socketio and self.__loop:
            future = asyncio.run_coroutine_threadsafe(
                self.socketio.emit("fsms_update", payload),
                self.__loop,
            )
            future.add_done_callback(self._handle_emit_result)


def main() -> None:
    rclpy.init()
    node = YasminViewerNode()

    try:
        node.start_backend_server()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
