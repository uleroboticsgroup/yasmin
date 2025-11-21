# Copyright (C) 2025 Miguel Ángel González Santamarta
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
import importlib
from typing import List, Optional
from lxml import etree as ET
from yasmin_pybind_bridge import CppStateFactory
from ament_index_python import get_package_share_path


class PluginInfo:

    def __init__(
        self,
        plugin_type: str,
        class_name: Optional[str] = None,
        module: Optional[str] = None,
        file_name: Optional[str] = None,
        package_name: Optional[str] = None,
    ) -> None:
        self._cpp_factory: CppStateFactory = CppStateFactory()

        self.plugin_type: str = plugin_type
        self.class_name: Optional[str] = class_name
        self.module: Optional[str] = module
        self.file_name: Optional[str] = file_name
        self.package_name: Optional[str] = package_name
        self.outcomes: List[str] = []

        if self.plugin_type == "python":
            loaded_module = importlib.import_module(self.module)
            state_class = getattr(loaded_module, self.class_name)
            self.outcomes = list(state_class().get_outcomes())
        elif self.plugin_type == "cpp":
            self.outcomes = list(self._cpp_factory.create(self.class_name).get_outcomes())
        elif self.plugin_type == "xml":
            package_path = get_package_share_path(self.package_name)
            file_path = ""
            for root, dirs, files in os.walk(package_path):
                if self.file_name in files:
                    file_path = os.path.join(root, self.file_name)
                    break
            if not file_path:
                raise ValueError(
                    f"Could not find XML file {self.file_name} in package {self.package_name}"
                )
            tree = ET.parse(file_path)
            root = tree.getroot()
            outcomes_str: str = root.attrib.get("outcomes", "")
            self.outcomes = outcomes_str.split() if outcomes_str else []
