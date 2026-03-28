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

from pathlib import Path
from xml.etree import ElementTree as ET

from yasmin_editor.io import model_from_xml

from yasmin_cli.completer import is_state_machine_xml, xml_file_completer


def add_print_verb(subparsers):
    parser = subparsers.add_parser(
        "print",
        help="Print a YASMIN state machine from an XML file",
        description="Print a YASMIN state machine from an XML file",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = xml_file_completer

    parser.set_defaults(main=_main_print)


def _main_print(args):
    xml_path = Path(args.state_machine_file)
    if not xml_path.is_file():
        print(f"File does not exist: {args.state_machine_file}")
        return 1

    if not is_state_machine_xml(xml_path):
        print(f"Not a valid YASMIN state machine XML file: {args.state_machine_file}")
        return 1

    try:
        model = model_from_xml(xml_path)
    except ET.ParseError as exc:
        print(f"Failed to parse XML file '{args.state_machine_file}': {exc}")
        return 1
    except ValueError as exc:
        print(str(exc))
        return 1

    print(model)
    return 0
