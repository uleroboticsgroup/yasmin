// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef YASMIN_ROS__GET_PARAMETERS_STATE_HPP_
#define YASMIN_ROS__GET_PARAMETERS_STATE_HPP_

#include <any>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"
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
   * @brief Type alias for a map of parameters.
   */
  using Parameters = std::unordered_map<std::string, std::any>;

  /**
   * @brief Shared pointer type for GetParametersState.
   */
  YASMIN_PTR_ALIASES(GetParametersState)

  /**
   * @brief Constructs a GetParametersState with a map of parameters.
   *
   * @param parameters A map of parameter names to their default values.
   * @param node A shared pointer to the ROS 2 node.
   */
  GetParametersState(const Parameters &parameters,
                     rclcpp::Node::SharedPtr node = nullptr);

  /**
   * @brief Executes the state to retrieve parameters.
   *
   * @param blackboard A reference to the Yasmin blackboard.
   * @return A string representing the outcome of the execution.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// @brief Map of parameters to retrieve, where the key is the parameter name
  /// and the value is the default value.
  std::unordered_map<std::string, std::any> parameters_;
  /// @brief Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__GET_PARAMETERS_STATE_HPP_
