// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN_ROS__TF_BUFFER_STATE_HPP_
#define YASMIN_ROS__TF_BUFFER_STATE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif

#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief State that creates a tf2 buffer and transform listener.
 *
 * The created objects are stored in the blackboard under the keys
 * "tf_buffer" and "tf_listener" so that following states can reuse the same
 * listener for transform lookups.
 */
class TfBufferState : public yasmin::State {
public:
  /**
   * @brief Shared pointer type for TfBufferState.
   */
  YASMIN_PTR_ALIASES(TfBufferState)

  /**
   * @brief Construct a new TfBufferState.
   */
  TfBufferState();

  /**
   * @brief Read the configured parameters.
   */
  void configure() override;

  /**
   * @brief Create the tf2 buffer/listener pair and store them in the
   * blackboard.
   *
   * @param blackboard Blackboard used to share the created objects.
   * @return The execution outcome.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  /// Node used to create the tf2 listener subscriptions.
  rclcpp::Node::SharedPtr node_;
  /// Buffer cache duration in seconds.
  double cache_time_sec_;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__TF_BUFFER_STATE_HPP_
