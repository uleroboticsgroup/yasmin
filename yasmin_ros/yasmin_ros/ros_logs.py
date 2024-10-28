# Copyright (C) 2024  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import yasmin
from yasmin_ros.yasmin_node import YasminNode


def set_ros_loggers() -> None:
    node = YasminNode.get_instance()
    yasmin.set_loggers(
        node.get_logger().info,
        node.get_logger().warn,
        node.get_logger().debug,
        node.get_logger().error,
    )
