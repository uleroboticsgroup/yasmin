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

import os

from ros2cli.command import CommandExtension

from yasmin_cli.verb.edit import add_edit_verb
from yasmin_cli.verb.info import add_info_verb
from yasmin_cli.verb.list import add_list_verb
from yasmin_cli.verb.print import add_print_verb
from yasmin_cli.verb.run import add_run_verb
from yasmin_cli.verb.test import add_test_verb
from yasmin_cli.verb.validate import add_validate_verb
from yasmin_cli.verb.viewer import add_viewer_verb


class YasminCommand(CommandExtension):
    """YASMIN command line tools."""

    def add_arguments(self, parser, cli_name):
        subparsers = parser.add_subparsers(dest="verb", metavar="verb")
        subparsers.required = True

        add_edit_verb(subparsers)
        add_info_verb(subparsers)
        add_list_verb(subparsers)
        add_print_verb(subparsers)
        add_run_verb(subparsers)
        add_test_verb(subparsers)
        add_validate_verb(subparsers)
        add_viewer_verb(subparsers)

        self._parser = parser

    def main(self, *, parser, args):
        if hasattr(args, "main"):
            result = args.main(args)
            os._exit(result if result is not None else 0)

        self._parser.print_help()
        os._exit(0)
