# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json

ALIAS_MAP = {
    "string": "str",
    "integer": "int",
    "double": "float",
    "boolean": "bool",
    "list[string]": "list[str]",
    "list[double]": "list[float]",
    "list[boolean]": "list[bool]",
    "dict[string,str]": "dict[str,str]",
    "dict[str,string]": "dict[str,str]",
    "dict[string,string]": "dict[str,str]",
    "dict[string,int]": "dict[str,int]",
    "dict[string,integer]": "dict[str,int]",
    "dict[str,integer]": "dict[str,int]",
    "dict[string,float]": "dict[str,float]",
    "dict[string,double]": "dict[str,float]",
    "dict[str,double]": "dict[str,float]",
    "dict[string,bool]": "dict[str,bool]",
    "dict[string,boolean]": "dict[str,bool]",
    "dict[str,boolean]": "dict[str,bool]",
}

VALID_TYPES = {
    "str",
    "int",
    "float",
    "bool",
    "list[str]",
    "list[int]",
    "list[float]",
    "list[bool]",
    "dict[str,str]",
    "dict[str,int]",
    "dict[str,float]",
    "dict[str,bool]",
}


def normalize_type(type_name: str) -> str:
    normalized = (type_name or "str").strip().lower().replace(" ", "")
    return ALIAS_MAP.get(normalized, normalized)


def validate_type(type_name: str) -> str:
    normalized = normalize_type(type_name)
    if normalized not in VALID_TYPES:
        raise ValueError(f"Unsupported default_type '{type_name}'")
    return normalized


def parse_bool_value(value_str: str) -> bool:
    normalized = value_str.strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"Invalid boolean default value '{value_str}'")


def load_json_value(value_str: str):
    try:
        return json.loads(value_str)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Invalid JSON default value '{value_str}' for container type"
        ) from exc


def parse_key_value(value_str: str, type_str: str):
    normalized_type = validate_type(type_str)

    if normalized_type == "str":
        return value_str
    if normalized_type == "int":
        return int(value_str)
    if normalized_type == "float":
        return float(value_str)
    if normalized_type == "bool":
        return parse_bool_value(value_str)
    if normalized_type.startswith("list["):
        return _parse_list_value(value_str, normalized_type)
    if normalized_type.startswith("dict["):
        return _parse_dict_value(value_str, normalized_type)

    raise ValueError(f"Unsupported default_type '{type_str}'")


def _parse_list_value(value_str: str, normalized_type: str):
    value = load_json_value(value_str)
    if not isinstance(value, list):
        raise ValueError(
            f"Default value '{value_str}' must decode to a JSON list for type {normalized_type}"
        )

    if normalized_type == "list[str]":
        if not all(isinstance(item, str) for item in value):
            raise ValueError(f"Type {normalized_type} expects only string entries")
        return value

    if normalized_type == "list[int]":
        if not all(
            isinstance(item, int) and not isinstance(item, bool) for item in value
        ):
            raise ValueError(f"Type {normalized_type} expects only integer entries")
        return value

    if normalized_type == "list[float]":
        if not all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            for item in value
        ):
            raise ValueError(f"Type {normalized_type} expects only numeric entries")
        return [float(item) for item in value]

    if normalized_type == "list[bool]":
        if not all(isinstance(item, bool) for item in value):
            raise ValueError(f"Type {normalized_type} expects only boolean entries")
        return value

    raise ValueError(f"Unsupported list default type '{normalized_type}'")


def _parse_dict_value(value_str: str, normalized_type: str):
    value = load_json_value(value_str)
    if not isinstance(value, dict):
        raise ValueError(
            f"Default value '{value_str}' must decode to a JSON object for type {normalized_type}"
        )

    if not all(isinstance(key, str) for key in value):
        raise ValueError(f"Type {normalized_type} expects only string keys")

    if normalized_type == "dict[str,str]":
        if not all(isinstance(item, str) for item in value.values()):
            raise ValueError(f"Type {normalized_type} expects only string values")
        return value

    if normalized_type == "dict[str,int]":
        if not all(
            isinstance(item, int) and not isinstance(item, bool)
            for item in value.values()
        ):
            raise ValueError(f"Type {normalized_type} expects only integer values")
        return value

    if normalized_type == "dict[str,float]":
        if not all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            for item in value.values()
        ):
            raise ValueError(f"Type {normalized_type} expects only numeric values")
        return {key: float(item) for key, item in value.items()}

    if normalized_type == "dict[str,bool]":
        if not all(isinstance(item, bool) for item in value.values()):
            raise ValueError(f"Type {normalized_type} expects only boolean values")
        return value

    raise ValueError(f"Unsupported dict default type '{normalized_type}'")


def format_default_value(value, type_name: str) -> str:
    normalized_type = normalize_type(type_name)

    if normalized_type == "bool":
        return "true" if value else "false"

    if normalized_type.startswith(("list[", "dict[")):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))

    return str(value)
