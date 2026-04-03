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

import argparse
import json

import rclpy
from rclpy.node import Node

from .plugin_manager import PluginManager


class DiscoveryNode(Node):
    """ROS node used to print discovered plugins."""

    def __init__(self) -> None:
        """Create the discovery node."""
        super().__init__("yasmin_plugins_discovery")


def parse_args():
    """Parse command line arguments for plugin discovery."""
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--force-refresh",
        action="store_true",
        help="Ignore cache and perform a full rescan.",
    )
    parser.add_argument(
        "--max-cache-age-sec",
        type=int,
        default=0,
        help="Invalidate cache after this age in seconds. 0 disables age-based invalidation.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print full plugin metadata.",
    )
    return parser.parse_args()


def _format_plugin_header(plugin) -> str:
    """Format the compact plugin header line."""
    if plugin.plugin_type == "python":
        source = f"module={plugin.module}"
    elif plugin.plugin_type == "cpp":
        source = f"package={plugin.package_name}"
    else:
        source = f"package={plugin.package_name}"

    return (
        f"[{plugin.plugin_type}] {plugin.display_name} | id={plugin.unique_id} | {source}"
    )


def _format_plugin_details(plugin) -> list[str]:
    """Format detailed plugin metadata lines."""
    details = [
        f"  class_name: {plugin.class_name}",
        f"  module: {plugin.module}",
        f"  file_name: {plugin.file_name}",
        f"  package_name: {plugin.package_name}",
        f"  relative_path: {getattr(plugin, 'relative_path', None)}",
        f"  description: {plugin.description or '-'}",
        f"  outcomes: {json.dumps(plugin.outcomes, ensure_ascii=False)}",
        f"  outcome_descriptions: {json.dumps(plugin.outcome_descriptions, ensure_ascii=False)}",
        f"  input_keys: {json.dumps(plugin.input_keys, ensure_ascii=False)}",
        f"  output_keys: {json.dumps(plugin.output_keys, ensure_ascii=False)}",
    ]
    return details


def _log_plugins(node: Node, title: str, plugins: list, verbose: bool) -> None:
    """Log a group of plugins."""
    node.get_logger().info(f"{title}: {len(plugins)}")

    for plugin in plugins:
        node.get_logger().info(_format_plugin_header(plugin))

        if not verbose:
            continue

        for line in _format_plugin_details(plugin):
            node.get_logger().info(line)


def main() -> int:
    """Run plugin discovery and print the discovered plugins."""
    args = parse_args()

    started_rclpy = False
    if not rclpy.ok():
        rclpy.init()
        started_rclpy = True

    node = DiscoveryNode()

    try:
        manager = PluginManager(max_cache_age_sec=args.max_cache_age_sec)
        manager.load_all_plugins(
            force_refresh=args.force_refresh,
        )

        _log_plugins(node, "C++ plugins", manager.cpp_plugins, args.verbose)
        _log_plugins(node, "Python plugins", manager.python_plugins, args.verbose)
        _log_plugins(node, "XML state machines", manager.xml_files, args.verbose)
    finally:
        node.destroy_node()
        if started_rclpy and rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
