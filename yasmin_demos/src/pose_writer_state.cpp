#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/pose_writer_state.h"
#include "yasmin_ros/ros_logs.hpp"

PoseWriterState::PoseWriterState() : yasmin::State({"outcome1"}) {}

std::string PoseWriterState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing state POSE_WRITER");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  rclcpp::SerializedMessage serialized_message;
  serializer.serialize_message(&pose, &serialized_message);

  const auto &rcl_serialized_message =
      serialized_message.get_rcl_serialized_message();

  std::vector<uint8_t> pose_bytes(rcl_serialized_message.buffer,
                                  rcl_serialized_message.buffer +
                                      rcl_serialized_message.buffer_length);

  blackboard->set<std::vector<uint8_t>>("pose_bytes", pose_bytes);
  blackboard->set<std::string>("pose_bytes__type", "geometry_msgs/msg/Pose");

  YASMIN_LOG_INFO("Stored serialized Pose with %zu bytes", pose_bytes.size());

  return "outcome1";
}

PoseWriterState::~PoseWriterState() {}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PoseWriterState, yasmin::State)
