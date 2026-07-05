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

import sys
from dataclasses import dataclass as _dataclass, field as _field


def dataclass(_cls=None, /, **kwargs):
    if "slots" in kwargs and sys.version_info < (3, 10):
        del kwargs["slots"]
    if _cls is not None:
        return _dataclass(_cls, **kwargs)
    return lambda cls: _dataclass(cls, **kwargs)


field = _field
