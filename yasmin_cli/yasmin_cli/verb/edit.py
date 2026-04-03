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
from pathlib import Path

from yasmin_cli.completer import is_state_machine_xml, xml_file_completer


def run_editor(state_machine_file: str = "") -> int:
    if state_machine_file:
        xml_path = Path(state_machine_file)

        if not xml_path.is_file():
            print(f"File does not exist: {state_machine_file}")
            return 1

        if not is_state_machine_xml(xml_path):
            print(f"Not a valid YASMIN state machine XML file: {state_machine_file}")
            return 1

    command = [
        "ros2",
        "run",
        "yasmin_editor",
        "yasmin_editor",
    ]

    if state_machine_file:
        command.extend(["--xml-file", state_machine_file])

    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


def add_edit_verb(subparsers):
    parser = subparsers.add_parser(
        "edit",
        help="Open a YASMIN XML state machine in the editor",
        description="Open a YASMIN XML state machine in the editor",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        nargs="?",
        default="",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = xml_file_completer

    parser.set_defaults(main=_main_edit)


def _main_edit(args):
    return run_editor(args.state_machine_file)
