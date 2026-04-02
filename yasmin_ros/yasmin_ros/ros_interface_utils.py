# Copyright (C) 2026 Maik Knof
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
