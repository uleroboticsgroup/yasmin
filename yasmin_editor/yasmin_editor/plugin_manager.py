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
from tqdm import tqdm
import xml.etree.ElementTree as ET

import rclpy
from yasmin_editor.plugin_info import PluginInfo
from yasmin import set_log_level, LogLevel
from ament_index_python import get_packages_with_prefixes, get_package_share_path


class PluginManager:

    def __init__(self) -> None:
        self.cpp_plugins = []
        self.python_plugins = []
        self.xml_files = []

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
        xml_file = os.path.join(package_share_path, "resource", "plugins.xml")

        if os.path.exists(xml_file):
            with open(xml_file, "r", encoding="utf-8") as f:
                content = f.read().strip()

            # Split the content by <library> tags while keeping the tags
            libraries_content = []
            start = 0

            while True:
                # Find the next <library> tag
                lib_start = content.find("<library", start)
                if lib_start == -1:
                    break

                # Find the closing </library> tag for this library
                lib_end = content.find("</library>", lib_start)
                if lib_end == -1:
                    break

                lib_end += len("</library>")  # Include the closing tag
                library_content = content[lib_start:lib_end]
                libraries_content.append(library_content)
                start = lib_end

            # Parse each library content separately
            for library_content in libraries_content:
                library_xml = f"<root>{library_content}</root>"
                root = ET.fromstring(library_xml)

                for library in root.findall("library"):
                    for class_elem in library.findall("class"):
                        class_name = class_elem.get("name")
                        self.load_cpp_plugin(class_name)

    def load_python_plugins_from_package(self, package_name: str) -> None:
        # Skip known packages that don't contain yasmin states
        skip_packages = {
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
            import importlib
            import inspect
            from yasmin import State

            # Try to import the package to get its path
            try:
                import io
                from contextlib import redirect_stdout, redirect_stderr

                with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    package = importlib.import_module(package_name)

            except (
                ImportError,
                ModuleNotFoundError,
                SystemExit,
                AttributeError,
                Exception,
            ):
                return

            # Get the package directory
            if not hasattr(package, "__file__") or package.__file__ is None:
                return

            package_path = os.path.dirname(package.__file__)

            # Walk through package files
            for dirpath, _, filenames in os.walk(package_path):
                # Skip __pycache__ and test directories
                if "__pycache__" in dirpath or "/test" in dirpath:
                    continue

                # Limit search depth
                depth = len(dirpath.replace(package_path, "").split(os.sep))
                if depth > 3:
                    continue

                for filename in filenames:
                    if not filename.endswith(".py") or filename == "__init__.py":
                        continue

                    file_path = os.path.join(dirpath, filename)

                    # Quick file content check for yasmin imports and State class
                    try:
                        with open(file_path, "r", encoding="utf-8") as f:
                            content = f.read(2000)  # Read only first 2000 chars

                        # Skip if no yasmin import or State reference
                        if "yasmin" not in content or "State" not in content:
                            continue

                    except (IOError, UnicodeDecodeError):
                        continue

                    # Now try to import the module
                    module_path = os.path.relpath(file_path, package_path)
                    module_name = module_path[:-3].replace(os.sep, ".")
                    full_module_name = f"{package_name}.{module_name}"

                    try:
                        with redirect_stdout(io.StringIO()), redirect_stderr(
                            io.StringIO()
                        ):
                            module = importlib.import_module(full_module_name)

                        # Find State subclasses
                        for name, obj in inspect.getmembers(module, inspect.isclass):
                            if (
                                obj.__module__ == full_module_name
                                and issubclass(obj, State)
                                and obj is not State
                            ):
                                self.load_python_plugin(full_module_name, name)

                    except (Exception, SystemExit):
                        pass

        except (Exception, SystemExit):
            pass

    def load_xml_state_machines_from_package(self, package_name: str) -> None:
        package_share_path = get_package_share_path(package_name)
        state_machines_path = os.path.join(package_share_path, "state_machines")

        if os.path.exists(state_machines_path):
            for filename in os.listdir(state_machines_path):
                if filename.endswith(".xml"):
                    xml_file = os.path.join(state_machines_path, filename)
                    self.load_xml_state_machine(xml_file)

    def load_cpp_plugin(self, class_name: str) -> None:
        plugin_info = PluginInfo(plugin_type="cpp", class_name=class_name)
        self.cpp_plugins.append(plugin_info)

    def load_python_plugin(self, module: str, class_name: str) -> None:
        plugin_info = PluginInfo(
            plugin_type="python", class_name=class_name, module=module
        )
        self.python_plugins.append(plugin_info)

    def load_xml_state_machine(self, xml_file: str) -> None:
        plugin_info = PluginInfo(plugin_type="xml", file_path=xml_file)
        self.xml_files.append(plugin_info)
