# Copyright (C) 2026 Maik Knof
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

from typing import Type

from rosidl_runtime_py.utilities import get_action, get_message, get_service

_ACTION_SUFFIXES = {
    "_Goal": "Goal",
    "_Result": "Result",
    "_Feedback": "Feedback",
}

_SERVICE_SUFFIXES = {
    "_Request": "Request",
    "_Response": "Response",
}


def resolve_interface_type(interface_type: str) -> Type:
    interface_type = interface_type.strip()
    if not interface_type:
        raise ValueError("The 'interface_type' parameter must not be empty.")

    if "/msg/" in interface_type:
        return get_message(interface_type)

    if "/srv/" in interface_type:
        for suffix, attribute_name in _SERVICE_SUFFIXES.items():
            if interface_type.endswith(suffix):
                service_type = get_service(interface_type[: -len(suffix)])
                return getattr(service_type, attribute_name)
        raise ValueError(
            "Service interface types must use the concrete request/response "
            "syntax, for example 'std_srvs/srv/Trigger_Request'."
        )

    if "/action/" in interface_type:
        for suffix, attribute_name in _ACTION_SUFFIXES.items():
            if interface_type.endswith(suffix):
                action_type = get_action(interface_type[: -len(suffix)])
                return getattr(action_type, attribute_name)
        raise ValueError(
            "Action interface types must use the concrete goal/result/feedback "
            "syntax, for example 'example_interfaces/action/Fibonacci_Goal'."
        )

    raise ValueError(
        "Unsupported interface type syntax. Expected '<pkg>/msg/<Type>', "
        "'<pkg>/srv/<Type>_Request', '<pkg>/srv/<Type>_Response', "
        "'<pkg>/action/<Type>_Goal', '<pkg>/action/<Type>_Result' or "
        "'<pkg>/action/<Type>_Feedback'."
    )
