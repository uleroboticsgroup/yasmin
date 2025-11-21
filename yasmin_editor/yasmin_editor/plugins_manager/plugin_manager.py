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
import io
import importlib
import inspect
from typing import List, Optional
from contextlib import redirect_stdout, redirect_stderr
from tqdm import tqdm
from lxml import etree as ET
import rclpy
import yasmin
from yasmin import State, set_log_level, LogLevel
from ament_index_python import get_packages_with_prefixes, get_package_share_path
from yasmin_editor.plugins_manager.plugin_info import PluginInfo


class PluginManager:

    def __init__(self) -> None:
        self.cpp_plugins: List[PluginInfo] = []
        self.python_plugins: List[PluginInfo] = []
        self.xml_files: List[PluginInfo] = []

    def load_all_plugins(self) -> None:
        set_log_level(LogLevel.WARN)
        packages = list(get_packages_with_prefixes().keys())

        for package in tqdm(packages, desc="Loading plugins"):
            self.load_cpp_plugins_from_package(package)
            self.load_python_plugins_from_package(package)
            self.load_xml_state_machines_from_package(package)

        if rclpy.ok():
            rclpy.shutdown()

    def load_cpp_plugins_from_package(self, package_name: str) -> None:
        package_share_path = get_package_share_path(package_name)

        for root, dirs, files in os.walk(package_share_path):
            for filename in files:
                if filename.endswith(".xml"):
                    xml_file = os.path.join(root, filename)
                    try:
                        with open(xml_file, "r", encoding="utf-8") as f:
                            content: str = f.read().strip()
                    except (IOError, UnicodeDecodeError):
                        continue

                    # Check if this XML contains plugin definitions
                    if "<library" in content and "yasmin" in content.lower():
                        libraries_content: List[str] = []
                        start: int = 0

                        while True:
                            lib_start: int = content.find("<library", start)
                            if lib_start == -1:
                                break

                            lib_end: int = content.find("</library>", lib_start)
                            if lib_end == -1:
                                break

                            lib_end += len("</library>")
                            libraries_content.append(content[lib_start:lib_end])
                            start = lib_end

                        for library_content in libraries_content:
                            library_xml: str = f"<root>{library_content}</root>"
                            root_elem = ET.fromstring(library_xml)

                            for library in root_elem.findall("library"):
                                for class_elem in library.findall("class"):
                                    class_name: str = class_elem.get("name")
                                    if class_name and "yasmin" in class_name.lower():
                                        self.load_cpp_plugin(class_name)

    def load_python_plugins_from_package(self, package_name: str) -> None:
        skip_packages: set = {
            "yasmin_ros",
            "rosidl_adapter",
            "rosidl_cli",
            "rosidl_generator_c",
            "rosidl_generator_cpp",
            "rosidl_generator_py",
            "rosidl_parser",
            "rosidl_pycommon",
            "rosidl_runtime_py",
            "rosidl_typesupport_c",
            "rosidl_typesupport_cpp",
            "rosidl_typesupport_introspection_c",
            "rosidl_typesupport_introspection_cpp",
        }

        if package_name in skip_packages or package_name.startswith("_"):
            return

        try:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                package = importlib.import_module(package_name)
        except Exception:
            return

        if not hasattr(package, "__file__") or package.__file__ is None:
            return

        package_path: str = os.path.dirname(package.__file__)

        for dirpath, _, filenames in os.walk(package_path):
            if "__pycache__" in dirpath or "/test" in dirpath:
                continue

            if len(dirpath.replace(package_path, "").split(os.sep)) > 3:
                continue

            for filename in filenames:
                if not filename.endswith(".py") or filename == "__init__.py":
                    continue

                file_path: str = os.path.join(dirpath, filename)

                try:
                    with open(file_path, "r", encoding="utf-8") as f:
                        content: str = f.read(2000)

                    if "yasmin" not in content or "State" not in content:
                        continue
                except (IOError, UnicodeDecodeError):
                    continue

                module_path: str = os.path.relpath(file_path, package_path)
                module_name: str = module_path[:-3].replace(os.sep, ".")
                full_module_name: str = f"{package_name}.{module_name}"

                try:
                    with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                        module = importlib.import_module(full_module_name)

                    for name, obj in inspect.getmembers(module, inspect.isclass):
                        if (
                            obj.__module__ == full_module_name
                            and issubclass(obj, State)
                            and obj is not State
                        ):
                            self.load_python_plugin(full_module_name, name)
                except Exception:
                    pass

    def load_xml_state_machines_from_package(self, package_name: str) -> None:
        package_share_path = get_package_share_path(package_name)

        for root, dirs, files in os.walk(package_share_path):
            for filename in files:
                if filename.endswith(".xml"):
                    xml_file: str = os.path.join(root, filename)
                    try:
                        tree = ET.parse(xml_file)
                        xml_root = tree.getroot()
                        if xml_root.tag == "StateMachine":
                            self.load_xml_state_machine(filename, package_name)
                    except (ET.ParseError, IOError):
                        continue

    def load_cpp_plugin(self, class_name: str) -> None:
        try:
            plugin_info: PluginInfo = PluginInfo(plugin_type="cpp", class_name=class_name)
        except Exception:
            yasmin.YASMIN_LOG_ERROR(f"Failed to load C++ plugin: {class_name}")
            return
        self.cpp_plugins.append(plugin_info)

    def load_python_plugin(self, module: str, class_name: str) -> None:
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="python", class_name=class_name, module=module
            )
        except Exception:
            yasmin.YASMIN_LOG_ERROR(
                f"Failed to load Python plugin: {class_name} from module {module}"
            )
            return
        self.python_plugins.append(plugin_info)

    def load_xml_state_machine(
        self, xml_file: str, package_name: Optional[str] = None
    ) -> None:
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="xml", file_name=xml_file, package_name=package_name
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load XML state machine: {xml_file} of package {package_name}. Error: "{e}"'
            )

            return
        self.xml_files.append(plugin_info)
