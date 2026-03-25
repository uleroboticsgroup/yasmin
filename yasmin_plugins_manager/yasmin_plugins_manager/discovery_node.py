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
    return parser.parse_args()


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

        node.get_logger().info(f"Found {len(manager.cpp_plugins)} C++ plugins")
        for plugin in manager.cpp_plugins:
            node.get_logger().info(
                f"[cpp] {plugin.display_name}  id={plugin.unique_id}  package={plugin.package_name}"
            )

        node.get_logger().info(f"Found {len(manager.python_plugins)} Python plugins")
        for plugin in manager.python_plugins:
            node.get_logger().info(
                f"[python] {plugin.display_name}  id={plugin.unique_id}  module={plugin.module}"
            )

        node.get_logger().info(f"Found {len(manager.xml_files)} XML state machines")
        for plugin in manager.xml_files:
            node.get_logger().info(
                f"[xml] {plugin.display_name}  id={plugin.unique_id}  package={plugin.package_name}"
            )
    finally:
        node.destroy_node()
        if started_rclpy and rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
