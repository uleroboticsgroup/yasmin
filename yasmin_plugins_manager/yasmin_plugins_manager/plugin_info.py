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
import os
import re
from typing import Dict, List, Optional

from ament_index_python import get_package_share_path
from lxml import etree as ET
from yasmin_pybind_bridge import CppStateFactory


class PluginInfo:
    """Store metadata of a discovered plugin."""

    @staticmethod
    def _split_cpp_template_args(args: str) -> list[str]:
        """Split a C++ template argument list at top-level commas."""
        parts: list[str] = []
        current: list[str] = []
        depth = 0

        for char in args:
            if char == "<":
                depth += 1
                current.append(char)
            elif char == ">":
                depth = max(0, depth - 1)
                current.append(char)
            elif char == "," and depth == 0:
                part = "".join(current).strip()
                if part:
                    parts.append(part)
                current = []
            else:
                current.append(char)

        tail = "".join(current).strip()
        if tail:
            parts.append(tail)

        return parts

    @staticmethod
    def _normalize_cpp_metadata_type(type_name: str) -> str:
        """Normalize common C++ metadata type names for user-facing output."""
        normalized_type = " ".join(type_name.strip().split())

        string_fragment_pattern = re.compile(
            r"std::(?:__cxx11::)?basic_string<char(?:,\s*std::char_traits<char>)?(?:,\s*std::allocator<char>)?\s*>"
        )
        integer_type_pattern = re.compile(
            r"^(unsigned\s+)?(short|int|long|long\s+long)(\s+int)?$"
        )

        normalized_type = re.sub(r"\s*<\s*", "<", normalized_type)
        normalized_type = re.sub(r"\s*>\s*", ">", normalized_type)
        normalized_type = re.sub(r"\s*,\s*", ", ", normalized_type)

        if normalized_type == "std::string":
            return "str"

        canonical_type = string_fragment_pattern.sub("std::string", normalized_type)

        if canonical_type == "std::string":
            return "str"

        if canonical_type == "bool":
            return "bool"

        if canonical_type in {"float", "double", "long double"}:
            return "float"

        if integer_type_pattern.match(canonical_type):
            return "int"

        if canonical_type.startswith("const "):
            return PluginInfo._normalize_cpp_metadata_type(canonical_type[6:])

        if canonical_type.endswith("&"):
            return PluginInfo._normalize_cpp_metadata_type(canonical_type[:-1].strip())

        if canonical_type.endswith("*"):
            pointee_type = PluginInfo._normalize_cpp_metadata_type(
                canonical_type[:-1].strip()
            )
            return f"{pointee_type}*"

        template_match = re.match(r"^([a-zA-Z0-9_:]+)<(.+)>$", canonical_type)
        if not template_match:
            return canonical_type

        template_name = template_match.group(1)
        template_args = PluginInfo._split_cpp_template_args(template_match.group(2))

        if template_name == "std::vector" and len(template_args) >= 1:
            element_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            return f"list[{element_type}]"

        if (
            template_name in {"std::unordered_map", "std::map"}
            and len(template_args) >= 2
        ):
            key_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            value_type = PluginInfo._normalize_cpp_metadata_type(template_args[1])
            return f"dict[{key_type}, {value_type}]"

        if (
            template_name in {"std::shared_ptr", "std::unique_ptr"}
            and len(template_args) >= 1
        ):
            return PluginInfo._normalize_cpp_metadata_type(template_args[0])

        if template_name == "std::optional" and len(template_args) >= 1:
            value_type = PluginInfo._normalize_cpp_metadata_type(template_args[0])
            return f"optional[{value_type}]"

        return canonical_type

    @staticmethod
    def _is_primitive_metadata_value(value) -> bool:
        """Return whether a metadata default value can be cached directly."""
        return value is None or isinstance(value, (str, int, float, bool))

    @staticmethod
    def _describe_metadata_value_type(value) -> str:
        """Return a stable type name for non-primitive metadata defaults."""
        return type(value).__name__

    @classmethod
    def _normalize_cpp_metadata_entries(cls, metadata_entries: List[dict]) -> List[dict]:
        """Normalize metadata dictionaries and sanitize non-serializable defaults."""
        normalized_entries: List[dict] = []

        for metadata_entry in metadata_entries:
            normalized_entry = dict(metadata_entry)

            default_value = normalized_entry.get("default_value")
            default_value_type = normalized_entry.get("default_value_type")

            if isinstance(default_value_type, str):
                normalized_entry["default_value_type"] = cls._normalize_cpp_metadata_type(
                    default_value_type
                )
            elif default_value is not None:
                normalized_entry["default_value_type"] = (
                    cls._describe_metadata_value_type(default_value)
                )

            if not cls._is_primitive_metadata_value(default_value):
                if not normalized_entry.get("default_value_type"):
                    normalized_entry["default_value_type"] = (
                        cls._describe_metadata_value_type(default_value)
                    )
                normalized_entry["default_value"] = ""

            normalized_entries.append(normalized_entry)

        return normalized_entries

    def __init__(
        self,
        plugin_type: str,
        class_name: Optional[str] = None,
        module: Optional[str] = None,
        file_name: Optional[str] = None,
        package_name: Optional[str] = None,
        relative_path: Optional[str] = None,
    ) -> None:
        """
        Load plugin metadata immediately.

        Parameters
        ----------
        plugin_type : str
            Plugin type identifier. Supported values are ``python``, ``cpp`` and ``xml``.
        class_name : Optional[str]
            Class or exported plugin name.
        module : Optional[str]
            Python module containing the state class.
        file_name : Optional[str]
            XML file name.
        package_name : Optional[str]
            Package containing the plugin.
        relative_path : Optional[str]
            Relative path of an XML file inside the package share directory.
        """
        self._cpp_factory: Optional[CppStateFactory] = CppStateFactory()

        self.plugin_type: str = plugin_type
        self.class_name: Optional[str] = class_name
        self.module: Optional[str] = module
        self.file_name: Optional[str] = file_name
        self.package_name: Optional[str] = package_name
        self.relative_path: Optional[str] = relative_path
        self.outcomes: List[str] = []
        self.description: str = ""
        self.outcome_descriptions: Dict[str, str] = {}
        self.input_keys: List[dict] = []
        self.output_keys: List[dict] = []
        self.parameters: List[dict] = []

        if self.plugin_type == "python":
            loaded_module = importlib.import_module(self.module)
            state_class = getattr(loaded_module, self.class_name)
            instance = state_class()
            self._load_instance_metadata(instance)

        elif self.plugin_type == "cpp":
            instance = self._cpp_factory.create(self.class_name)
            self._load_instance_metadata(instance)

        elif self.plugin_type == "xml":
            package_path = get_package_share_path(self.package_name)

            if self.relative_path:
                file_path = os.path.join(package_path, self.relative_path)
            else:
                file_path = ""
                for root, _, files in os.walk(package_path):
                    if self.file_name in files:
                        file_path = os.path.join(root, self.file_name)
                        self.relative_path = os.path.relpath(file_path, package_path)
                        break

            if not file_path or not os.path.exists(file_path):
                raise ValueError(
                    f"Could not find XML file {self.file_name} in package {self.package_name}"
                )

            if self.file_name is None:
                self.file_name = os.path.basename(file_path)

            tree = ET.parse(file_path)
            root = tree.getroot()
            outcomes_str: str = root.attrib.get("outcomes", "")
            self.outcomes = outcomes_str.split() if outcomes_str else []
            self.description = root.attrib.get("description", "")
            self.outcome_descriptions = {}

            for outcome_elem in root.findall("FinalOutcome"):
                outcome_name = outcome_elem.attrib.get("name", "")
                outcome_description = outcome_elem.attrib.get("description", "")
                if outcome_name and outcome_description:
                    self.outcome_descriptions[outcome_name] = outcome_description

            for param_elem in root.findall("Param"):
                param_name = param_elem.attrib.get("name", "")
                if not param_name:
                    continue

                param_data = {
                    "name": param_name,
                    "description": param_elem.attrib.get("description", ""),
                    "default_value_type": param_elem.attrib.get("default_type", "str"),
                    "default_value": param_elem.attrib.get("default_value", ""),
                    "has_default": "default_value" in param_elem.attrib,
                }

                self.parameters.append(dict(param_data))

            for key_elem in root.findall("Key"):
                key_name = key_elem.attrib.get("name", "")
                if not key_name:
                    continue

                key_data = {
                    "name": key_name,
                    "description": key_elem.attrib.get("description", ""),
                    "default_value_type": key_elem.attrib.get("default_type", "str"),
                    "default_value": key_elem.attrib.get("default_value", ""),
                    "has_default": "default_value" in key_elem.attrib,
                }

                key_type = key_elem.attrib.get("type", "in")
                if key_type in ("in", "in/out"):
                    self.input_keys.append(dict(key_data))
                if key_type in ("out", "in/out"):
                    self.output_keys.append(dict(key_data))

            if not self.input_keys and not self.output_keys:
                for default_elem in root.findall("Default"):
                    key_name = default_elem.attrib.get("key", "")
                    if not key_name:
                        continue
                    self.input_keys.append(
                        {
                            "name": key_name,
                            "description": default_elem.attrib.get("description", ""),
                            "default_value_type": default_elem.attrib.get("type", "str"),
                            "default_value": default_elem.attrib.get("value", ""),
                            "has_default": True,
                        }
                    )

    def _load_instance_metadata(self, instance) -> None:
        """
        Load metadata from a Python or C++ state instance.

        Each metadata field is queried independently so one failing accessor
        does not suppress the remaining metadata.
        """
        try:
            self.outcomes = list(instance.get_outcomes())
        except Exception:
            self.outcomes = []

        try:
            self.description = instance.get_description()
        except Exception:
            self.description = ""

        try:
            self.outcome_descriptions = instance.get_outcome_descriptions()
        except Exception:
            self.outcome_descriptions = {}

        try:
            self.input_keys = list(instance.get_input_keys())
            self.input_keys = self._normalize_cpp_metadata_entries(self.input_keys)
        except Exception:
            self.input_keys = []

        try:
            self.output_keys = list(instance.get_output_keys())
            self.output_keys = self._normalize_cpp_metadata_entries(self.output_keys)
        except Exception:
            self.output_keys = []

        try:
            self.parameters = list(instance.get_parameters())
            self.parameters = self._normalize_cpp_metadata_entries(self.parameters)
        except Exception:
            self.parameters = []

    def to_cache_dict(self) -> dict:
        """Serialize the full plugin metadata for caching."""
        return {
            "plugin_type": self.plugin_type,
            "class_name": self.class_name,
            "module": self.module,
            "file_name": self.file_name,
            "package_name": self.package_name,
            "relative_path": self.relative_path,
            "outcomes": self.outcomes,
            "description": self.description,
            "outcome_descriptions": self.outcome_descriptions,
            "input_keys": self.input_keys,
            "output_keys": self.output_keys,
            "parameters": self.parameters,
        }

    @classmethod
    def from_cache_dict(cls, data: dict) -> "PluginInfo":
        """
        Create a plugin info object from cached metadata.

        No plugin factory instance is created for cached entries.
        """
        instance = cls.__new__(cls)
        instance._cpp_factory = None
        instance.plugin_type = data.get("plugin_type")
        instance.class_name = data.get("class_name")
        instance.module = data.get("module")
        instance.file_name = data.get("file_name")
        instance.package_name = data.get("package_name")
        instance.relative_path = data.get("relative_path")
        instance.outcomes = list(data.get("outcomes", []))
        instance.description = data.get("description", "")
        instance.outcome_descriptions = dict(data.get("outcome_descriptions", {}))
        instance.input_keys = list(data.get("input_keys", []))
        instance.output_keys = list(data.get("output_keys", []))
        instance.parameters = list(data.get("parameters", []))

        instance.input_keys = cls._normalize_cpp_metadata_entries(instance.input_keys)
        instance.output_keys = cls._normalize_cpp_metadata_entries(instance.output_keys)
        instance.parameters = cls._normalize_cpp_metadata_entries(instance.parameters)

        return instance

    @property
    def display_name(self) -> str:
        """Return the display name of the plugin."""
        if self.plugin_type == "python":
            return self.class_name or ""
        if self.plugin_type == "cpp":
            return self.class_name or ""
        return self.file_name or ""

    @property
    def unique_id(self) -> str:
        """Return a stable unique identifier for the plugin."""
        if self.plugin_type == "python":
            return f"python:{self.module}:{self.class_name}"
        if self.plugin_type == "cpp":
            return f"cpp:{self.class_name}"
        return f"xml:{self.package_name}:{self.file_name}"

    @property
    def dedup_key(self) -> str:
        """Return a stable key used for discovery deduplication."""
        if self.plugin_type == "python":
            return f"python:{self.module}:{self.class_name}"
        if self.plugin_type == "cpp":
            return f"cpp:{self.class_name}"
        return f"xml:{self.package_name}:{self.relative_path or self.file_name}"
