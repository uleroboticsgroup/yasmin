// Copyright (C) 2025 Miguel Ángel González Santamarta
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
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_ROS__GET_PARAMETER_STATE_HPP
#define YASMIN_ROS__GET_PARAMETER_STATE_HPP

#include <any>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief State that retrieves parameters from the ROS 2 parameter server.
 *
 * This state retrieves parameters from the ROS 2 parameter server and stores
 * them in the blackboard.
 */
class GetParametersState : public yasmin::State {
public:
  /**
   * @brief Constructs a GetParametersState with a map of parameters.
   *
   * @param parameters A map of parameter names to their default values.
   * @param node A shared pointer to the ROS 2 node.
   */
  GetParametersState(const std::map<std::string, std::any> &parameters,
                     rclcpp::Node::SharedPtr node = nullptr);

  /**
   * @brief Executes the state to retrieve parameters.
   *
   * @param blackboard A reference to the Yasmin blackboard.
   * @return A string representing the outcome of the execution.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override;

private:
  /// Map of parameters to retrieve, where the key is the parameter name
  /// and the value is the default value.
  std::map<std::string, std::any> parameters_;
  /// Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__GET_PARAMETER_STATE_HPP
