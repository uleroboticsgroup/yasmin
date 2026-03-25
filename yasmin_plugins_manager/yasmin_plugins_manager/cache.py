#!/usr/bin/env python3

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

import hashlib
import json
import os
import platform
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python import get_packages_with_prefixes

CACHE_VERSION = 1


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


def build_environment_fingerprint() -> Dict[str, Any]:
    """Build a fingerprint for the active ROS and Python environment."""
    packages = get_packages_with_prefixes()
    payload = {
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", ""),
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}",
        "platform": platform.platform(),
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
