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

#ifndef YASMIN_DEMOS_POSE_WRITER_STATE_H
#define YASMIN_DEMOS_POSE_WRITER_STATE_H

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
  ~PoseWriterState();

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
  double position_x_;
  double position_y_;
  double position_z_;
  double orientation_w_;
};

#endif // YASMIN_DEMOS_POSE_WRITER_STATE_H
