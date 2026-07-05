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

import hashlib
import json
import os
import platform
import re
import sys
from pathlib import Path
from typing import Any, Dict, Optional, List

from ament_index_python import get_packages_with_prefixes

CACHE_VERSION = 4
IGNORE_PACKAGES_ENV_VAR = "YASMIN_DISCOVERY_IGNORE_PACKAGES"


def get_default_cache_dir() -> Path:
    """Return the default cache directory."""
    return Path(
        os.environ.get(
            "YASMIN_CACHE",
            os.path.join(
                os.path.expanduser("~"),
                ".cache",
                "yasmin_plugins_manager",
            ),
        )
    )


def ensure_cache_dir(cache_dir: Optional[Path] = None) -> Path:
    """Create the cache directory if it does not exist."""
    path = cache_dir or get_default_cache_dir()
    path.mkdir(parents=True, exist_ok=True)
    return path


def get_cache_file(cache_dir: Optional[Path] = None) -> Path:
    """Return the cache file path."""
    return ensure_cache_dir(cache_dir) / "plugins_cache.json"


def get_ignored_packages_from_env() -> List[str]:
    """Return the sorted package ignore list configured through the environment."""
    raw_value = os.environ.get(IGNORE_PACKAGES_ENV_VAR, "")
    if not raw_value.strip():
        return []

    normalized_value = raw_value
    for separator in [",", ";", os.pathsep]:
        normalized_value = normalized_value.replace(separator, " ")

    packages = {
        item.strip() for item in re.split(r"\s+", normalized_value) if item.strip()
    }
    return sorted(packages)


def build_environment_fingerprint() -> Dict[str, Any]:
    """Build a fingerprint for the active ROS and Python environment."""
    packages = get_packages_with_prefixes()
    ignored_packages = get_ignored_packages_from_env()
    payload = {
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", ""),
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}",
        "platform": platform.platform(),
        "ignored_packages_env": ignored_packages,
        "packages": dict(sorted(packages.items())),
    }
    payload_json = json.dumps(payload, sort_keys=True)
    payload_hash = hashlib.sha256(payload_json.encode("utf-8")).hexdigest()
    return {
        "hash": payload_hash,
        "payload": payload,
    }


def load_cache(cache_dir: Optional[Path] = None) -> Optional[Dict[str, Any]]:
    """Load the cache file if it exists."""
    cache_file = get_cache_file(cache_dir)
    if not cache_file.exists():
        return None

    try:
        with cache_file.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except Exception:
        return None


def save_cache(data: Dict[str, Any], cache_dir: Optional[Path] = None) -> None:
    """Write the cache file atomically."""
    cache_file = get_cache_file(cache_dir)
    tmp_file = cache_file.with_suffix(".tmp")

    with tmp_file.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)

    tmp_file.replace(cache_file)


def stat_signature(path: str) -> Optional[Dict[str, Any]]:
    """Return a small signature for a file or directory."""
    try:
        stat_result = os.stat(path)
    except OSError:
        return None

    return {
        "path": path,
        "mtime_ns": stat_result.st_mtime_ns,
        "size": stat_result.st_size,
    }


def is_stat_signature_valid(signature: Dict[str, Any]) -> bool:
    """Check whether a cached file signature is still valid."""
    current = stat_signature(signature["path"])
    if current is None:
        return False

    return current["mtime_ns"] == signature.get("mtime_ns") and current[
        "size"
    ] == signature.get("size")
