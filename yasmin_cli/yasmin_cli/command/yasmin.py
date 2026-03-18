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

import os

from ros2cli.command import CommandExtension

from yasmin_cli.verb.edit import add_edit_verb
from yasmin_cli.verb.info import add_info_verb
from yasmin_cli.verb.list import add_list_verb
from yasmin_cli.verb.run import add_run_verb
from yasmin_cli.verb.test import add_test_verb


class YasminCommand(CommandExtension):
    """YASMIN command line tools."""

    def add_arguments(self, parser, cli_name):
        subparsers = parser.add_subparsers(dest="verb", metavar="verb")
        subparsers.required = True

        add_edit_verb(subparsers)
        add_info_verb(subparsers)
        add_list_verb(subparsers)
        add_run_verb(subparsers)
        add_test_verb(subparsers)

        self._parser = parser

    def main(self, *, parser, args):
        if hasattr(args, "main"):
            result = args.main(args)
            os._exit(result if result is not None else 0)

        self._parser.print_help()
        os._exit(0)
