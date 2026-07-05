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
