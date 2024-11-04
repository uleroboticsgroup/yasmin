// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS_BASIC_OUTCOME_HPP
#define YASMIN_ROS_BASIC_OUTCOME_HPP

#include <string>

namespace yasmin_ros {

/**
 * @namespace yasmin_ros::basic_outcomes
 * @brief Provides common outcome constants for ROS 2.
 *
 * The basic_outcomes namespace defines constants for common action outcomes
 * within the yasmin_ros framework. These constants represent the status of
 * actions or tasks, making it easier to standardize result handling.
 */
namespace basic_outcomes {

/**
 * @brief Constant representing a successful action outcome.
 *
 * This string constant is used to indicate that an action has successfully
 * completed without any issues.
 */
constexpr char SUCCEED[] = "succeeded";

/**
 * @brief Constant representing an aborted action outcome.
 *
 * This string constant signifies that an action was aborted, likely due to an
 * error or unexpected condition that prevented its completion.
 */
constexpr char ABORT[] = "aborted";

/**
 * @brief Constant representing a canceled action outcome.
 *
 * This string constant is used when an action is manually canceled by a user
 * or system before it could complete.
 */
constexpr char CANCEL[] = "canceled";

/**
 * @brief Constant representing a timed-out action outcome.
 *
 * This string constant is used to indicate that an action did not complete
 * within the allocated time, resulting in a timeout status.
 */
constexpr char TIMEOUT[] = "timeout";

} // namespace basic_outcomes
} // namespace yasmin_ros

#endif // YASMIN_ROS_BASIC_OUTCOME_HPP