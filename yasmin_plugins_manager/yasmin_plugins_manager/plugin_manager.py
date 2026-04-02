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
from ament_index_python import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_path,
    get_packages_with_prefixes,
)
from ament_index_python.resources import (
    get_resource,
    get_resource_types,
    get_resources,
    has_resource,
)
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
    """Discover and cache available YASMIN plugins."""

    def __init__(
        self,
        cache_dir: Optional[Path] = None,
        max_cache_age_sec: int = 0,
    ) -> None:
        """
        Initialize the plugin manager.

        Parameters
        ----------
        cache_dir : Optional[Path]
            Optional custom cache directory.
        max_cache_age_sec : int
            Maximum cache age in seconds. A value of 0 disables age-based invalidation.
        """
        self.cache_dir = cache_dir
        self.max_cache_age_sec = max_cache_age_sec
        self.cpp_plugins: List[PluginInfo] = []
        self.python_plugins: List[PluginInfo] = []
        self.xml_files: List[PluginInfo] = []

    def load_all_plugins(
        self,
        hide_progress: bool = False,
        force_refresh: bool = False,
    ) -> None:
        """
        Load all plugins from cache or perform a full discovery run.

        The cache is used when possible. If the cache is missing, outdated or invalid,
        a full discovery is performed and the result is written back to the cache.
        """
        set_log_level(LogLevel.WARN)

        if not force_refresh and self._load_from_cache():
            return

        self.cpp_plugins = []
        self.python_plugins = []
        self.xml_files = []

        tracked_files: List[dict] = []
        tracked_dirs: List[dict] = []
        packages = list(get_packages_with_prefixes().keys())
        cpp_resource_map = self._get_cpp_plugin_resource_map()

        for package in tqdm(packages, desc="Loading plugins", disable=hide_progress):
            try:
                package_share_path = get_package_share_path(package)
                share_signature = stat_signature(str(package_share_path))
                if share_signature:
                    tracked_dirs.append(share_signature)
            except Exception:
                pass

            self.load_cpp_plugins_from_package(
                package_name=package,
                tracked_files=tracked_files,
                cpp_resource_map=cpp_resource_map,
            )
            self.load_python_plugins_from_package(package, tracked_files, tracked_dirs)
            self.load_xml_state_machines_from_package(package, tracked_files)

        self._save_to_cache(tracked_files, tracked_dirs)

    def _load_from_cache(self) -> bool:
        """
        Load cached plugins if the cache is still valid.

        Returns
        -------
        bool
            True if the cache was successfully loaded, otherwise False.
        """
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
        """
        Save the discovered plugin metadata to the cache.

        Parameters
        ----------
        tracked_files : List[dict]
            File signatures used to invalidate the cache when discovery-relevant files change.
        tracked_dirs : List[dict]
            Directory signatures used to invalidate the cache when package contents change.
        """
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

    def _is_plugin_resource_type(self, resource_type: str) -> bool:
        """
        Check whether an ament resource type belongs to pluginlib.

        Parameters
        ----------
        resource_type : str
            Ament resource type name.

        Returns
        -------
        bool
            True if the resource type contains pluginlib plugin exports.
        """
        return "__pluginlib__plugin" in resource_type

    def _get_registered_plugin_resource_list(self) -> list[str]:
        """
        Return all pluginlib-related resource types from the ament index.

        Returns
        -------
        list[str]
            List of pluginlib resource type names.
        """
        return list(filter(self._is_plugin_resource_type, get_resource_types()))

    def _get_cpp_plugin_resource_map(self) -> dict[str, list[str]]:
        """
        Build a mapping from package name to exported plugin XML resource paths.

        Returns
        -------
        dict[str, list[str]]
            Mapping from package name to plugin XML paths relative to the package prefix.
        """
        resource_map: dict[str, list[str]] = {}

        for plugin_resource in self._get_registered_plugin_resource_list():
            for package_name in get_resources(plugin_resource).keys():
                resource_map.setdefault(package_name, [])

                try:
                    component_registry, _ = get_resource(plugin_resource, package_name)
                except Exception:
                    continue

                resource_map[package_name] += [
                    line.split(";")[0]
                    for line in component_registry.splitlines()
                    if line.strip()
                ]

        return resource_map

    def load_cpp_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
        cpp_resource_map: Optional[dict[str, list[str]]] = None,
    ) -> None:
        """
        Discover YASMIN C++ plugins from pluginlib exports.

        This uses the ament resource index. Only plugin classes exported with base class
        ``yasmin::State`` are loaded.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of parsed plugin XML files.
        cpp_resource_map : Optional[dict[str, list[str]]]
            Optional precomputed map of package names to plugin XML resource paths.
        """
        if cpp_resource_map is None:
            cpp_resource_map = self._get_cpp_plugin_resource_map()

        resource_paths = cpp_resource_map.get(package_name, [])
        if not resource_paths:
            return

        try:
            package_prefix = get_package_prefix(package_name)
        except PackageNotFoundError:
            return

        for resource_path in resource_paths:
            plugin_xml = os.path.join(package_prefix, resource_path)

            if not os.path.isfile(plugin_xml):
                continue

            if tracked_files is not None:
                signature = stat_signature(plugin_xml)
                if signature:
                    tracked_files.append(signature)

            try:
                tree = ET.parse(plugin_xml)
            except ET.ParseError:
                continue

            for elem in tree.iter():
                if elem.tag != "class":
                    continue

                base_class_type = elem.attrib.get("base_class_type", "")
                if base_class_type != "yasmin::State":
                    continue

                plugin_type = elem.attrib.get("name") or elem.attrib.get("type")
                if not plugin_type:
                    continue

                self.load_cpp_plugin(plugin_type)

    def load_python_plugins_from_package(
        self,
        package_name: str,
        tracked_files: Optional[List[dict]] = None,
        tracked_dirs: Optional[List[dict]] = None,
    ) -> None:
        """
        Discover Python plugins by scanning a Python package.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of relevant Python files.
        tracked_dirs : Optional[List[dict]]
            Optional list that receives the package directory signature.
        """
        skip_packages: set = {
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
        """
        Discover XML state machines from a package share directory.

        Parameters
        ----------
        package_name : str
            Package name to inspect.
        tracked_files : Optional[List[dict]]
            Optional list that receives signatures of relevant XML files.
        """
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

                        relative_path = os.path.relpath(xml_file, package_share_path)
                        self.load_xml_state_machine(
                            filename,
                            package_name,
                            relative_path=relative_path,
                        )
                except (ET.ParseError, IOError):
                    continue

    def load_cpp_plugin(self, class_name: str) -> None:
        """
        Load one C++ plugin.

        Parameters
        ----------
        class_name : str
            Fully qualified exported C++ class name.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="cpp",
                class_name=class_name,
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load C++ plugin: {class_name}. Error: "{e}"'
            )
            return
        self.cpp_plugins.append(plugin_info)

    def load_python_plugin(self, module: str, class_name: str) -> None:
        """
        Load one Python plugin.

        Parameters
        ----------
        module : str
            Python module containing the state class.
        class_name : str
            Python class name of the state.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="python",
                class_name=class_name,
                module=module,
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load Python plugin: {class_name} from module {module}. Error: "{e}"'
            )
            return
        self.python_plugins.append(plugin_info)

    def load_xml_state_machine(
        self,
        xml_file: str,
        package_name: Optional[str] = None,
        relative_path: Optional[str] = None,
    ) -> None:
        """
        Load one XML state machine.

        Parameters
        ----------
        xml_file : str
            File name of the XML state machine.
        package_name : Optional[str]
            Package that contains the XML state machine.
        relative_path : Optional[str]
            Relative path inside the package share directory.
        """
        try:
            plugin_info: PluginInfo = PluginInfo(
                plugin_type="xml",
                file_name=xml_file,
                package_name=package_name,
                relative_path=relative_path,
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f'Failed to load XML state machine: {xml_file} of package {package_name}. Error: "{e}"'
            )
            return
        self.xml_files.append(plugin_info)
