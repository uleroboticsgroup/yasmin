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
};

#endif // YASMIN_DEMOS_POSE_WRITER_STATE_H
