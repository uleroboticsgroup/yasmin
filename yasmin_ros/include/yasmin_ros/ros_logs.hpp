// Copyright (C) 2024  Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_ROS_LOGS_HPP
#define YASMIN_ROS_LOGS_HPP

namespace yasmin_ros {

/**
 * @brief Sets the logging functions for ROS, linking ROS log levels to YASMIN
 * loggers.
 *
 * This function configures YASMIN to use ROS logging mechanisms for error,
 * warning, info, and debug levels, ensuring messages from YASMIN are routed
 * through ROS loggers.
 *
 * @note This function should be called once to set up the loggers before any
 * logging occurs within YASMIN components.
 */
void set_ros_loggers();

} // namespace yasmin_ros

#endif // YASMIN_ROS_LOGS_HPP
