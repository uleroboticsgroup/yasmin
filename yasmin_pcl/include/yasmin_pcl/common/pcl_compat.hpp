// Copyright (C) 2026 Maik Knof
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YASMIN_PCL__COMMON__PCL_COMPAT_HPP_
#define YASMIN_PCL__COMMON__PCL_COMPAT_HPP_

// pcl/types.h (providing pcl::Indices, pcl::IndicesPtr, etc.) was introduced
// in PCL 1.12.  Older releases that ship with ROS 2 Galactic / Foxy only
// have PCL 1.10 where these types live in pcl/pcl_base.h instead.

#include <pcl/pcl_config.h>

#if PCL_MAJOR_VERSION > 1 || (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 12)
#include <pcl/types.h>
#else
#include <pcl/pcl_base.h>
#endif

#endif // YASMIN_PCL__COMMON__PCL_COMPAT_HPP_
