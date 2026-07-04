# Copyright (C) 2023 Miguel Ángel González Santamarta
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

"""
This module defines a set of status codes that can be used to represent
the outcome of various operations within the application. These codes can
be utilized for error handling, logging, and user feedback.
"""

# Constants representing the various status codes
SUCCEED = "succeeded"  ##< Indicates that an operation has completed successfully.
ABORT = "aborted"  ##< Indicates that an operation was aborted before completion.
FAIL = "failed"  ##< Indicates that an operation has not achieved its intended result.
CANCEL = "canceled"  ##< Indicates that an operation was canceled by the user or system.
TIMEOUT = "timeout"  ##< Indicates that an operation has timed out and did not complete in a timely manner.
RETRY = "retry"  ##< Indicates that an operation has not achieved its intended result and the operation is going to be restarted.
