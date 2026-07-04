# Copyright (C) 2026 Maik Knof
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

from __future__ import annotations

import subprocess


def run_viewer(
    host: str = "127.0.0.1",
    port: int = 5000,
    msg_per_second: int = 30,
) -> int:
    command = [
        "ros2",
        "run",
        "yasmin_viewer",
        "yasmin_viewer_node",
        "--ros-args",
        "-p",
        f"host:={host}",
        "-p",
        f"port:={port}",
        "-p",
        f"msg_per_second:={msg_per_second}",
    ]

    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


def add_viewer_verb(subparsers):
    parser = subparsers.add_parser(
        "viewer",
        help="Start the YASMIN viewer",
        description="Start the YASMIN viewer web server",
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Viewer host address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Viewer port",
    )
    parser.add_argument(
        "--msg-per-second",
        type=int,
        default=30,
        help="Viewer update rate",
    )

    parser.set_defaults(main=_main_viewer)


def _main_viewer(args):
    return run_viewer(
        host=args.host,
        port=args.port,
        msg_per_second=args.msg_per_second,
    )
