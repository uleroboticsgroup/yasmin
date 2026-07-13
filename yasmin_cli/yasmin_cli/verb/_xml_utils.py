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

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List


def parse_assignments(values: List[str], assignment_kind: str) -> Dict[str, str]:
    result: Dict[str, str] = {}

    for value in values:
        if "=" not in value:
            raise ValueError(
                f"Invalid {assignment_kind} assignment '{value}'. Expected NAME=VALUE."
            )

        key, raw_value = value.split("=", 1)
        key = key.strip()

        if not key:
            raise ValueError(
                f"Invalid {assignment_kind} assignment '{value}'. Empty name is not allowed."
            )

        result[key] = raw_value

    return result


def indent_xml(element: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "    "
    child_indent = "\n" + (level + 1) * "    "

    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent

        for child in element:
            indent_xml(child, level + 1)

        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = indent

    if level and (not element.tail or not element.tail.strip()):
        element.tail = indent
