// Copyright (C) 2026 YASMIN Contributors
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

#ifndef YASMIN_DEMOS_SHARE_DIRECTORY_HPP_
#define YASMIN_DEMOS_SHARE_DIRECTORY_HPP_

#include <filesystem>
#include <string>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(32, 0, 0)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace yasmin_demos {

/**
 * @brief Resolves a relative path to an absolute path in the package's share
 * directory.
 *
 * This function uses ament_index_cpp to locate the share directory for the
 * yasmin_demos package and appends the provided relative path.
 *
 * @param relative_path Relative file path within the share directory.
 * @return std::string The absolute file path.
 */
inline std::string get_share_file_path(const std::string &relative_path) {
#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(32, 0, 0)
  return (ament_index_cpp::get_package_share_path("yasmin_demos") /
          relative_path)
      .string();
#else
  return ament_index_cpp::get_package_share_directory("yasmin_demos") + "/" +
         relative_path;
#endif
#else
  return ament_index_cpp::get_package_share_directory("yasmin_demos") + "/" +
         relative_path;
#endif
}

} // namespace yasmin_demos

#endif // YASMIN_DEMOS_SHARE_DIRECTORY_HPP_
