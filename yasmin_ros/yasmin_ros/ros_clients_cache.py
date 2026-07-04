# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from threading import RLock
from typing import Any, Dict, Tuple, Type, Union

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile

import yasmin


class ROSClientsCache:
    """
    Centralized cache for managing ROS 2 client objects.

    This class provides a thread-safe cache for storing and reusing ROS 2 client
    objects such as publishers, service clients, and action clients.
    This helps reduce resource usage and improves performance by avoiding duplicate
    creation of client objects.

    The cache is organized by client type and uses unique keys based on:
    - Node name
    - Message/Service/Action type
    - Topic/Service/Action name
    - Callback group name
    - QoS profile
    """

    ## Cache for action clients
    ## Key: (node_name, action_type_name, action_name, callback_group_name)
    ## Value: ActionClient instance
    _action_clients: Dict[Tuple[str, str, str, str], ActionClient] = {}

    ## Cache for service clients
    ## Key: (node_name, service_type_name, service_name, callback_group_name)
    ## Value: Client instance
    _service_clients: Dict[Tuple[str, str, str, str], Client] = {}

    ## Cache for publishers
    ## Key: (node_name, msg_type_name, topic_name, qos_hash)
    ## Value: Publisher instance
    _publishers: Dict[Tuple[str, str, str, str], Publisher] = {}

    ## Lock for thread-safe access to all caches
    _lock: RLock = RLock()

    @classmethod
    def _get_or_create(cls, cache, cache_key, log_label, factory):
        with cls._lock:
            existing = cache.get(cache_key)
            if existing is not None:
                yasmin.YASMIN_LOG_INFO(f"Reusing existing {log_label}")
                return existing
            yasmin.YASMIN_LOG_INFO(f"Creating new {log_label}")
            obj = factory()
            cache[cache_key] = obj
            return obj

    @classmethod
    def _clear(cls, cache, label):
        with cls._lock:
            cache.clear()
            yasmin.YASMIN_LOG_INFO(f"{label} cache cleared")

    @classmethod
    def _count(cls, cache):
        with cls._lock:
            return len(cache)

    @classmethod
    def get_or_create_action_client(
        cls,
        node: Node,
        action_type: Type,
        action_name: str,
        callback_group: CallbackGroup = None,
    ) -> ActionClient:
        """
        Get an existing action client from the cache or create a new one.

        Args:
            node (Node): The ROS 2 node to use.
            action_type (Type): The type of the action.
            action_name (str): The name of the action.
            callback_group (CallbackGroup, optional): The callback group for the action client.

        Returns:
            ActionClient: The cached or newly created action client.
        """
        node_name = node.get_name()
        action_type_name = f"{action_type.__module__}.{action_type.__name__}"
        callback_group_name = cls._get_callback_group_name(callback_group)
        cache_key = (node_name, action_type_name, action_name, callback_group_name)
        return cls._get_or_create(
            cls._action_clients,
            cache_key,
            f"action client for '{action_name}' of type '{action_type_name}'",
            lambda: ActionClient(
                node, action_type, action_name, callback_group=callback_group
            ),
        )

    @classmethod
    def get_or_create_service_client(
        cls,
        node: Node,
        service_type: Type,
        service_name: str,
        callback_group: CallbackGroup = None,
    ) -> Client:
        """
        Get an existing service client from the cache or create a new one.

        Args:
            node (Node): The ROS 2 node to use.
            service_type (Type): The type of the service.
            service_name (str): The name of the service.
            callback_group (CallbackGroup, optional): The callback group for the service client.

        Returns:
            Client: The cached or newly created service client.
        """
        node_name = node.get_name()
        service_type_name = f"{service_type.__module__}.{service_type.__name__}"
        callback_group_name = cls._get_callback_group_name(callback_group)
        cache_key = (node_name, service_type_name, service_name, callback_group_name)
        return cls._get_or_create(
            cls._service_clients,
            cache_key,
            f"service client for '{service_name}' of type '{service_type_name}'",
            lambda: node.create_client(
                service_type, service_name, callback_group=callback_group
            ),
        )

    @classmethod
    def get_or_create_publisher(
        cls,
        node: Node,
        msg_type: Type,
        topic_name: str,
        qos_profile: Union[QoSProfile, int] = 10,
        callback_group: CallbackGroup = None,
    ) -> Publisher:
        """
        Get an existing publisher from the cache or create a new one.

        Args:
            node (Node): The ROS 2 node to use.
            msg_type (Type): The type of the message.
            topic_name (str): The name of the topic.
            qos_profile (QoSProfile, optional): The QoS profile for the publisher.
            callback_group (CallbackGroup, optional): The callback group for the publisher.

        Returns:
            Publisher: The cached or newly created publisher.
        """
        node_name = node.get_name()
        msg_type_name = f"{msg_type.__module__}.{msg_type.__name__}"
        qos_hash = str(cls._hash_qos_profile(qos_profile))
        cache_key = (node_name, msg_type_name, topic_name, qos_hash)
        return cls._get_or_create(
            cls._publishers,
            cache_key,
            f"publisher for topic '{topic_name}' of type '{msg_type_name}'",
            lambda: node.create_publisher(
                msg_type, topic_name, qos_profile, callback_group=callback_group
            ),
        )

    @classmethod
    def clear_action_clients(cls) -> None:
        """
        Clear the action clients cache.
        """
        cls._clear(cls._action_clients, "Action clients")

    @classmethod
    def clear_service_clients(cls) -> None:
        """
        Clear the service clients cache.
        """
        cls._clear(cls._service_clients, "Service clients")

    @classmethod
    def clear_publishers(cls) -> None:
        """
        Clear the publishers cache.
        """
        cls._clear(cls._publishers, "Publishers")

    @classmethod
    def clear_all(cls) -> None:
        """
        Clear all client caches.
        """
        with cls._lock:
            cls._action_clients.clear()
            cls._service_clients.clear()
            cls._publishers.clear()
            yasmin.YASMIN_LOG_INFO("All ROS clients caches cleared")

    @classmethod
    def get_action_clients_count(cls) -> int:
        """
        Get the number of cached action clients.

        Returns:
            int: The number of cached action clients.
        """
        return cls._count(cls._action_clients)

    @classmethod
    def get_service_clients_count(cls) -> int:
        """
        Get the number of cached service clients.

        Returns:
            int: The number of cached service clients.
        """
        return cls._count(cls._service_clients)

    @classmethod
    def get_publishers_count(cls) -> int:
        """
        Get the number of cached publishers.

        Returns:
            int: The number of cached publishers.
        """
        return cls._count(cls._publishers)

    @classmethod
    def get_cache_stats(cls) -> Dict[str, int]:
        """
        Get statistics about all caches.

        Returns:
            Dict[str, int]: A dictionary with cache statistics.
        """
        with cls._lock:
            return {
                "action_clients": len(cls._action_clients),
                "service_clients": len(cls._service_clients),
                "publishers": len(cls._publishers),
                "total": len(cls._action_clients)
                + len(cls._service_clients)
                + len(cls._publishers),
            }

    @staticmethod
    def _hash_qos_profile(qos_profile: Any) -> int:
        """
        Create a hash for a QoS profile.

        Args:
            qos_profile: The QoS profile to hash (can be int or QoSProfile).

        Returns:
            int: A hash value for the QoS profile.
        """
        if isinstance(qos_profile, int):
            return qos_profile

        # For QoSProfile objects, create a hash from key attributes
        if hasattr(qos_profile, "depth"):
            return hash(
                (
                    qos_profile.history,
                    qos_profile.depth,
                    qos_profile.reliability,
                    qos_profile.durability,
                    (
                        qos_profile.deadline.nanoseconds
                        if hasattr(qos_profile.deadline, "nanoseconds")
                        else 0
                    ),
                    (
                        qos_profile.lifespan.nanoseconds
                        if hasattr(qos_profile.lifespan, "nanoseconds")
                        else 0
                    ),
                    qos_profile.liveliness,
                )
            )

        # Default hash
        return hash(str(qos_profile))

    @staticmethod
    def _get_callback_group_name(callback_group: CallbackGroup) -> str:
        """
        Get a string representation of a callback group.

        Args:
            callback_group: The callback group to get the name from.

        Returns:
            str: A string representation of the callback group.
        """
        if callback_group is None:
            return "none"

        # Try to get a meaningful name or use the type name
        if hasattr(callback_group, "__class__"):
            return f"{callback_group.__class__.__name__}_{id(callback_group)}"

        return str(id(callback_group))
