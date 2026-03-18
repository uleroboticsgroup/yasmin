#!/usr/bin/env python3

# Copyright (C) 2026 Maik Knof
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
