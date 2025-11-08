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

import importlib
import xml.etree.ElementTree as ET
from yasmin_pybind_bridge import CppStateFactory


class PluginInfo:

    def __init__(
        self,
        plugin_type: str,
        class_name: str = None,
        module: str = None,
        file_path: str = None,
    ) -> None:
        self._cpp_factory = CppStateFactory()

        self.plugin_type = plugin_type
        self.class_name = class_name
        self.module = module
        self.file_path = file_path

        # Get outcomes
        if self.plugin_type == "python":
            module = importlib.import_module(self.module)
            state_class = getattr(module, self.class_name)
            self.outcomes = state_class().get_outcomes()

        elif self.plugin_type == "cpp":
            self.outcomes = self._cpp_factory.create(self.class_name).get_outcomes()

        elif self.plugin_type == "xml":
            tree = ET.parse(self.file_path)
            root = tree.getroot()
            self.outcomes = root.attrib.get("outcomes", "").split(" ")
