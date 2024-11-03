# Copyright (C) 2023  Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
This module defines a set of status codes that can be used to represent 
the outcome of various operations within the application. These codes can 
be utilized for error handling, logging, and user feedback.
"""

# Constants representing the various status codes
SUCCEED = "succeeded"  ##< Indicates that an operation has completed successfully.
ABORT = "aborted"  ##< Indicates that an operation was aborted before completion.
CANCEL = "canceled"  ##< Indicates that an operation was canceled by the user or system.
TIMEOUT = "timeout"  ##< Indicates that an operation has timed out and did not complete in a timely manner.
