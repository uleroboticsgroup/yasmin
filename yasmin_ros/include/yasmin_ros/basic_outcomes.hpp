// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__BASIC_OUTCOMES_HPP_
#define YASMIN_ROS__BASIC_OUTCOMES_HPP_

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
inline constexpr char SUCCEED[] = "succeeded";

/**
 * @brief Constant representing an aborted action outcome.
 *
 * This string constant signifies that an action was aborted, likely due to an
 * error or unexpected condition that prevented its completion.
 */
inline constexpr char ABORT[] = "aborted";

/**
 * @brief Constant representing a failed action outcome.
 *
 * This string constant is used to indicate that an action has completed
 * but resulted in failure, meaning it did not achieve its intended result.
 */
inline constexpr char FAIL[] = "failed";

/**
 * @brief Constant representing a canceled action outcome.
 *
 * This string constant is used when an action is manually canceled by a user
 * or system before it could complete.
 */
inline constexpr char CANCEL[] = "canceled";

/**
 * @brief Constant representing a timed-out action outcome.
 *
 * This string constant is used to indicate that an action did not complete
 * within the allocated time, resulting in a timeout status.
 */
inline constexpr char TIMEOUT[] = "timeout";

/**
 * @brief Constant representing a retry action outcome.
 *
 * This string constant is used to indicate that a retry should be performed.
 */
inline constexpr char RETRY[] = "retry";
} // namespace basic_outcomes
} // namespace yasmin_ros

#endif // YASMIN_ROS__BASIC_OUTCOMES_HPP_