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


import time
import json
from threading import Thread
from typing import List, Dict

import rclpy
from rclpy.node import Node
import ament_index_python

from yasmin_msgs.msg import (
    StateMachine,
    State,
    Transition
)

from flask import Flask
from waitress import serve
from expiringdict import ExpiringDict


class YasminFsmViewer(Node):

    def __init__(self) -> None:

        super().__init__("yasmin_viewer")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("host", "localhost"),
                ("port", 5000),
            ]
        )

        self.__started = False
        self.__fsm_dict = ExpiringDict(max_len=300, max_age_seconds=3)

        thread_subscriber = Thread(target=self.start_subscriber)
        thread_subscriber.start()

        self.start_backend_server()

    def start_backend_server(self) -> None:
        app = Flask("yasmin_viewer",
                    static_folder=ament_index_python.get_package_share_directory(
                        "yasmin_viewer") + "/yasmin_viewer_web_client",
                    static_url_path="/")
        # app.config["ENV"] = "development"

        @app.route("/")
        def index():
            return app.send_static_file("index.html")

        @app.route("/get_fsms", methods=["GET"])
        def get_fsms():
            return json.dumps(self.__fsm_dict)

        @app.route("/get_fsm/<fsm_name>",  methods=["GET"])
        def get_fsm(fsm_name):

            if fsm_name in self.__fsm_dict:
                return json.dumps(self.__fsm_dict[fsm_name])

            return json.dumps({})

        self.__started = True

        # app.run(host="localhost", port=5000)

        _host = str(self.get_parameter('host').value)
        _port = int(self.get_parameter('port').value)
        print(f"Started Yasmin viewer on http://{_host}:{str(_port)}")
        serve(app,
              host=_host,
              port=_port)

    def start_subscriber(self) -> None:
        self.create_subscription(
            StateMachine,
            "/fsm_viewer",
            self.fsm_viewer_cb,
            10
        )

        rclpy.spin(self)

    def transition_msg_to_dict(self, tansitions: List[Transition]) -> Dict:
        transition_dict = {}

        for transition in tansitions:
            transition_dict[transition.outcome] = transition.state

        return transition_dict

    def state_msg_to_dict(self, msg: State) -> Dict:
        state_dict = {
            "id": msg.id,
            "parent": msg.parent,
            "name": msg.name,
            "transitions": self.transition_msg_to_dict(msg.transitions),
            "outcomes": msg.outcomes,
            "is_fsm": False,
            "is_fsm": msg.is_fsm,
            "current_state": msg.current_state
        }
        return state_dict

    def msg_to_dict(self,  msg: StateMachine) -> Dict:

        states_dict = []

        for state in msg.states:
            states_dict.append(self.state_msg_to_dict(state))

        return states_dict

    def fsm_viewer_cb(self, msg: StateMachine) -> None:

        while not self.__started:
            time.sleep(0.05)

        if msg.states:
            self.__fsm_dict[msg.states[0].name] = self.msg_to_dict(msg)


def main():
    rclpy.init()
    YasminFsmViewer()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
