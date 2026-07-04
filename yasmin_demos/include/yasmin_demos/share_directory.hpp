// Copyright (C) 2026 YASMIN Contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
#elif RCLCPP_VERSION_GTE(29, 5, 1)
  std::filesystem::path p;
  ament_index_cpp::get_package_share_directory("yasmin_demos", p);
  return (p / relative_path).string();
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
