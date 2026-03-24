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

from yasmin_cli.completer import find_plugin, plugin_completer, plugin_id


def _print_outcome_block(
    title: str, outcomes: list[str], outcome_descriptions: dict[str, str]
) -> None:
    print(f"{title}:")
    for outcome in outcomes:
        line = f" - {outcome}"
        desc = outcome_descriptions.get(outcome, "")
        if desc:
            line += f": {desc}"
        print(line)


def _print_key_block(title: str, keys: list[dict]) -> None:
    if not keys:
        return

    print(f"{title}:")
    for key in keys:
        name = key.get("name", "")
        description = key.get("description", "")
        has_default = key.get("has_default", False)

        line = f"  - {name}"
        if has_default:
            default_value = key.get("default_value")
            default_type = key.get("default_value_type", "unknown")
            line += f" [default={default_value!r}, type={default_type}]"
        print(line)

        if description:
            print(f"      {description}")


def _print_plugin_details(plugin) -> None:
    print(f"Plugin:      {plugin_id(plugin)}")
    print(f"Type:        {plugin.plugin_type}")
    print(f"Description: {plugin.description if plugin.description else '-'}")
    _print_outcome_block(f"Outcomes", plugin.outcomes, plugin.outcome_descriptions)
    _print_key_block("Input keys", plugin.input_keys)
    _print_key_block("Output keys", plugin.output_keys)


def add_info_verb(subparsers):
    parser = subparsers.add_parser(
        "info",
        help="Show detailed information for one YASMIN plugin",
        description="Show detailed metadata for one discovered YASMIN plugin",
    )

    plugin_arg = parser.add_argument(
        "plugin_id",
        help="Exact plugin id",
    )
    plugin_arg.completer = plugin_completer

    parser.set_defaults(main=_main_info)


def _main_info(args):
    plugin = find_plugin(args.plugin_id, include_xml=True)
    if plugin is None:
        print(f"Plugin not found: {args.plugin_id}")
        return 1

    _print_plugin_details(plugin)
    return 0
