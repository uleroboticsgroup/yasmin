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

from yasmin_cli.completer import filter_plugins, load_plugins, plugin_id


def _plugin_summary(plugin) -> str:
    return (
        f"[{plugin.plugin_type:<6}] {plugin_id(plugin)} | "
        f"outcomes={len(plugin.outcomes)}, "
        f"inputs={len(plugin.input_keys)}, "
        f"outputs={len(plugin.output_keys)}"
    )


def add_list_verb(subparsers):
    parser = subparsers.add_parser(
        "list",
        help="List discovered YASMIN plugins",
        description="List discovered YASMIN states, XML state machines, and metadata",
    )

    parser.add_argument(
        "--type",
        choices=["all", "cpp", "python", "xml"],
        default="all",
        help="Filter plugin type",
    )
    parser.add_argument(
        "--search",
        default=None,
        help="Filter by substring",
    )

    parser.set_defaults(main=_main_list)


def _main_list(args):
    plugins = filter_plugins(load_plugins(include_xml=True), args.type, args.search)

    if not plugins:
        print("No plugins found.")
        return 0

    for plugin in plugins:
        print(_plugin_summary(plugin))
    return 0
