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
import inspect
import io
import os
import time
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import List, Optional

import yasmin
from ament_index_python import get_package_share_path, get_packages_with_prefixes
from lxml import etree as ET
from tqdm import tqdm
from yasmin import LogLevel, State, set_log_level

from yasmin_plugins_manager.cache import (
    CACHE_VERSION,
    build_environment_fingerprint,
    is_stat_signature_valid,
    load_cache,
    save_cache,
    stat_signature,
)
from yasmin_plugins_manager.plugin_info import PluginInfo


class PluginManager:
    """Discovers and stores all available YASMIN plugins."""

    def __init__(
        self,
        cache_dir: Optional[Path] = None,
        max_cache_age_sec: int = 0,
    ) -> None:
        """Initialize the plugin manager."""
        self.cache_dir = cache_dir
        self.max_cache_age_sec = max_cache_age_sec
        self.cpp_plugins: List[PluginInfo] = []
        self.python_plugins: List[PluginInfo] = []
        self.xml_files: List[PluginInfo] = []

    def load_all_plugins(
        self,
        hide_progress: bool = False,
        force_refresh: bool = False,
        preload_metadata: bool = False,
    ) -> None:
        """Load plugins from cache or perform a full discovery."""
        del preload_metadata

        set_log_level(LogLevel.WARN)

        if not force_refresh and self._load_from_cache():
            return

        self.cpp_plugins = []
        self.python_plugins = []
        self.xml_files = []

        tracked_files: List[dict] = []
        tracked_dirs: List[dict] = []
        packages = list(get_packages_with_prefixes().keys())

        for package in tqdm(packages, desc="Loading plugins", disable=hide_progress):
            try:
                package_share_path = get_package_share_path(package)
                share_signature = stat_signature(str(package_share_path))
                if share_signature:
                    tracked_dirs.append(share_signature)
            except Exception:
                pass

            self.load_cpp_plugins_from_package(package, tracked_files)
            self.load_python_plugins_from_package(package, tracked_files, tracked_dirs)
            self.load_xml_state_machines_from_package(package, tracked_files)

        self._save_to_cache(tracked_files, tracked_dirs)

    def _load_from_cache(self) -> bool:
        """Load cached plugins if the cache is still valid."""
        cache = load_cache(self.cache_dir)
        if cache is None:
            return False

        if cache.get("cache_version") != CACHE_VERSION:
            return False

        if self.max_cache_age_sec > 0:
            created_at = cache.get("created_at", 0.0)
            if time.time() - created_at > self.max_cache_age_sec:
                return False

        current_env = build_environment_fingerprint()
        if cache.get("environment_hash") != current_env["hash"]:
            return False

        for signature in cache.get("tracked_files", []):
            if not is_stat_signature_valid(signature):
                return False

        for signature in cache.get("tracked_dirs", []):
            if not is_stat_signature_valid(signature):
                return False

        self.cpp_plugins = [
            PluginInfo.from_cache_dict(data) for data in cache.get("cpp_plugins", [])
        ]
        self.python_plugins = [
            PluginInfo.from_cache_dict(data) for data in cache.get("python_plugins", [])
        ]
        self.xml_files = [
            PluginInfo.from_cache_dict(data) for data in cache.get("xml_files", [])
        ]
        return True

    def _save_to_cache(
        self,
        tracked_files: List[dict],
        tracked_dirs: List[dict],
    ) -> None:
        """Save the discovered plugin metadata to the cache."""
        environment = build_environment_fingerprint()
        data = {
            "cache_version": CACHE_VERSION,
            "created_at": time.time(),
            "environment_hash": environment["hash"],
            "tracked_files": tracked_files,
            "tracked_dirs": tracked_dirs,
            "cpp_plugins": [plugin.to_cache_dict() for plugin in self.cpp_plugins],
            "python_plugins": [plugin.to_cache_dict() for plugin in self.python_plugins],
            "xml_files": [plugin.to_cache_dict() for plugin in self.xml_files],
        }
        save_cache(data, self.cache_dir)

    def load_cpp_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
    ) -> None:
        """Discover C++ plugins from a package share directory."""
        package_share_path = get_package_share_path(package_name)

        for root, dirs, files in os.walk(package_share_path):
            for filename in files:
                if not filename.endswith(".xml"):
                    continue

                xml_file = os.path.join(root, filename)
                try:
                    with open(xml_file, "r", encoding="utf-8") as f:
                        content: str = f.read().strip()
                except (IOError, UnicodeDecodeError):
                    continue

                if "<library" not in content or "yasmin" not in content.lower():
                    continue

                if tracked_files is not None:
                    signature = stat_signature(xml_file)
                    if signature:
                        tracked_files.append(signature)

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
                    try:
                        library_xml: str = f"<root>{library_content}</root>"
                        root_elem = ET.fromstring(library_xml)
                    except Exception:
                        continue

                    for library in root_elem.findall("library"):
                        for class_elem in library.findall("class"):
                            base_class_type = class_elem.get("base_class_type")
                            if base_class_type == "yasmin::State":
                                class_name: str = class_elem.get("name")
                                if class_name:
                                    self.load_cpp_plugin(class_name)

    def load_python_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
        tracked_dirs: Optional[List[dict]] = None,
    ) -> None:
        """Discover Python plugins by scanning a Python package."""
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

        if tracked_dirs is not None:
            signature = stat_signature(package_path)
            if signature:
                tracked_dirs.append(signature)

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

                if tracked_files is not None:
                    signature = stat_signature(file_path)
                    if signature:
                        tracked_files.append(signature)

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

    def load_xml_state_machines_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
    ) -> None:
        """Discover XML state machines from a package share directory."""
        package_share_path = get_package_share_path(package_name)

        for root, dirs, files in os.walk(package_share_path):
            for filename in files:
                if not filename.endswith(".xml"):
                    continue

                xml_file: str = os.path.join(root, filename)
                try:
                    tree = ET.parse(xml_file)
                    xml_root = tree.getroot()
                    if xml_root.tag == "StateMachine":
                        if tracked_files is not None:
                            signature = stat_signature(xml_file)
                            if signature:
                                tracked_files.append(signature)
                        self.load_xml_state_machine(filename, package_name)
                except (ET.ParseError, IOError):
                    continue

    def load_cpp_plugin(self, class_name: str) -> None:
        """Load one C++ plugin."""
        try:
            plugin_info: PluginInfo = PluginInfo(plugin_type="cpp", class_name=class_name)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load C++ plugin: {class_name}. Error: "{e}"'
            )
            return
        self.cpp_plugins.append(plugin_info)

    def load_python_plugin(self, module: str, class_name: str) -> None:
        """Load one Python plugin."""
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="python", class_name=class_name, module=module
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load Python plugin: {class_name} from module {module}. Error: "{e}"'
            )
            return
        self.python_plugins.append(plugin_info)

    def load_xml_state_machine(
        self, xml_file: str, package_name: Optional[str] = None
    ) -> None:
        """Load one XML state machine."""
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
