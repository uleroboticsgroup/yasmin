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

#ifndef YASMIN_DEMOS_POSE_WRITER_STATE_HPP_
#define YASMIN_DEMOS_POSE_WRITER_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

/**
 * @brief Represents a state that creates and serializes a Pose message.
 *
 * This state creates a geometry_msgs::msg::Pose message, serializes it to a
 * byte array, and stores both the serialized data and the type string in the
 * blackboard.
 */
class PoseWriterState : public yasmin::State {
public:
  /**
   * @brief Constructs a PoseWriterState object.
   */
  PoseWriterState();

  /**
   * @brief Destructs the PoseWriterState object.
   */
  ~PoseWriterState() override = default;

  /**
   * @brief Configures the state-local parameters.
   */
  void configure() override;

  /**
   * @brief Executes the Pose writer state logic.
   *
   * This method creates a Pose message, serializes it, stores the serialized
   * bytes in the blackboard under "pose_bytes", and stores the type string
   * under "pose_bytes__type".
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1".
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard);

private:
  /// @brief X coordinate of the pose
  double position_x_;
  /// @brief Y coordinate of the pose
  double position_y_;
  /// @brief Z coordinate of the pose
  double position_z_;
  /// @brief W component of the orientation quaternion
  double orientation_w_;
};

#endif // YASMIN_DEMOS_POSE_WRITER_STATE_HPP_
