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

#include "yasmin_ros/tf_buffer_state.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#if __has_include(<tf2/time.hpp>)
#include <tf2/time.hpp>
#else
#include <tf2/time.h>
#endif

#include "yasmin/logs.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace yasmin_ros {
namespace {

tf2::Duration to_tf_duration(const double seconds) {
  return std::chrono::duration_cast<tf2::Duration>(
      std::chrono::duration<double>(seconds));
}

} // namespace

TfBufferState::TfBufferState()
    : yasmin::State({basic_outcomes::SUCCEED, basic_outcomes::ABORT}),
      node_(nullptr), cache_time_sec_(10.0) {
  this->set_description(
      "Creates a shared tf2 buffer and transform listener and writes them to "
      "blackboard keys 'tf_buffer' and 'tf_listener'. Following states can "
      "reuse these objects to perform transform lookups.");
  this->set_outcome_description(
      basic_outcomes::SUCCEED,
      "The tf2 buffer and transform listener were created successfully.");
  this->set_outcome_description(
      basic_outcomes::ABORT,
      "Failed to create the tf2 buffer or transform listener.");
  this->add_output_key(
      "tf_buffer", "Shared pointer to the created tf2_ros::Buffer instance.");
  this->add_output_key(
      "tf_listener",
      "Shared pointer to the created tf2_ros::TransformListener instance.");
  this->declare_parameter<double>(
      "cache_time_sec",
      "How many seconds of transform history the tf2 buffer should keep.",
      10.0);
}

void TfBufferState::configure() {
  this->cache_time_sec_ = this->get_parameter<double>("cache_time_sec");

  if (!this->node_) {
    this->node_ = YasminNode::get_instance();
  }
}

std::string TfBufferState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  try {
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
        this->node_->get_clock(), to_tf_duration(this->cache_time_sec_));

    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer, this->node_, false);

    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
    blackboard->set<std::shared_ptr<tf2_ros::TransformListener>>("tf_listener",
                                                                 tf_listener);
    return basic_outcomes::SUCCEED;
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("TfBufferState failed to create tf2 objects: %s",
                     e.what());
    return basic_outcomes::ABORT;
  }
}

} // namespace yasmin_ros

PLUGINLIB_EXPORT_CLASS(yasmin_ros::TfBufferState, yasmin::State)
