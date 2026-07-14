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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_msgs/action/run_state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

namespace yasmin_factory {

class YasminFactoryActionServer {
public:
  using RunStateMachine = yasmin_msgs::action::RunStateMachine;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunStateMachine>;

  explicit YasminFactoryActionServer(
      const yasmin_ros::YasminNode::SharedPtr &node)
      : node_(node) {
    runtime_initializer_ = std::make_unique<YasminFactory>();
    node_->declare_parameter("state_machine_file", "");
    default_state_machine_file_ =
        node_->get_parameter("state_machine_file").as_string();

    node_->declare_parameter("enable_viewer_pub", true);
    enable_viewer_pub_ = node_->get_parameter("enable_viewer_pub").as_bool();

    action_server_ = rclcpp_action::create_server<RunStateMachine>(
        node_, "run_state_machine",
        std::bind(&YasminFactoryActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&YasminFactoryActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&YasminFactoryActionServer::handle_accepted, this,
                  std::placeholders::_1));

    YASMIN_LOG_INFO("yasmin_factory_action_server");
  }

  ~YasminFactoryActionServer() { shutdown(); }

  void shutdown() {
    std::shared_ptr<YasminFactory> factory;
    yasmin::StateMachine::SharedPtr state_machine;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (shutting_down_) {
        return;
      }
      shutting_down_ = true;
      cancel_requested_ = true;
      state_machine = active_state_machine_;
      factory = active_factory_;
    }

    if (state_machine) {
      state_machine->cancel_state_machine();
    }
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }

    action_server_.reset();
    runtime_initializer_.reset();
  }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const RunStateMachine::Goal> goal) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    const std::string state_machine_file = goal->state_machine_file.empty()
                                               ? default_state_machine_file_
                                               : goal->state_machine_file;
    if (shutting_down_ || busy_ || state_machine_file.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    busy_ = true;
    cancel_requested_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> &) {
    std::shared_ptr<YasminFactory> factory;
    yasmin::StateMachine::SharedPtr state_machine;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!busy_) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      cancel_requested_ = true;
      state_machine = active_state_machine_;
      factory = active_factory_;
    }

    if (state_machine) {
      state_machine->cancel_state_machine();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ =
        std::thread(&YasminFactoryActionServer::execute, this, goal_handle);
  }

  void publish_feedback(const std::shared_ptr<GoalHandle> &goal_handle,
                        const std::string &current_state) const {
    auto feedback = std::make_shared<RunStateMachine::Feedback>();
    feedback->current_state = current_state;
    goal_handle->publish_feedback(feedback);
  }

  bool
  cancellation_requested(const std::shared_ptr<GoalHandle> &goal_handle) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return cancel_requested_ || goal_handle->is_canceling();
  }

  void clear_active_state_machine() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_state_machine_.reset();
    active_factory_.reset();
  }

  void make_available() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    busy_ = false;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<RunStateMachine::Result>();
    const auto goal = goal_handle->get_goal();
    const std::string state_machine_file = goal->state_machine_file.empty()
                                               ? default_state_machine_file_
                                               : goal->state_machine_file;

    try {
      auto factory = std::make_shared<YasminFactory>();
      auto state_machine = factory->create_sm_from_file(state_machine_file);
      state_machine->set_sigint_handler(false);

      state_machine->add_start_cb(
          [this, goal_handle](const yasmin::Blackboard::SharedPtr &,
                              const std::string &start_state) {
            publish_feedback(goal_handle, start_state);
          });
      state_machine->add_transition_cb(
          [this, goal_handle](const yasmin::Blackboard::SharedPtr &,
                              const std::string &, const std::string &to_state,
                              const std::string &) {
            publish_feedback(goal_handle, to_state);
          });

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_factory_ = factory;
        active_state_machine_ = state_machine;
      }

      if (cancellation_requested(goal_handle)) {
        state_machine->cancel_state_machine();
        clear_active_state_machine();
        make_available();
        goal_handle->canceled(result);
        return;
      }

      std::unique_ptr<yasmin_viewer::YasminViewerPub> viewer;
      if (enable_viewer_pub_) {
        viewer =
            std::make_unique<yasmin_viewer::YasminViewerPub>(state_machine);
      }

      result->outcome = (*state_machine)();
      clear_active_state_machine();

      if (cancellation_requested(goal_handle)) {
        result->outcome.clear();
        make_available();
        goal_handle->canceled(result);
      } else {
        make_available();
        goal_handle->succeed(result);
      }
    } catch (const std::exception &error) {
      clear_active_state_machine();
      const bool canceled = cancellation_requested(goal_handle);
      result->outcome.clear();
      make_available();

      if (canceled) {
        goal_handle->canceled(result);
      } else {
        YASMIN_LOG_ERROR("State machine execution failed: %s", error.what());
        goal_handle->abort(result);
      }
    } catch (...) {
      clear_active_state_machine();
      const bool canceled = cancellation_requested(goal_handle);
      result->outcome.clear();
      make_available();

      if (canceled) {
        goal_handle->canceled(result);
      } else {
        YASMIN_LOG_ERROR("State machine execution failed with unknown error");
        goal_handle->abort(result);
      }
    }
  }

  yasmin_ros::YasminNode::SharedPtr node_;
  std::unique_ptr<YasminFactory> runtime_initializer_;
  rclcpp_action::Server<RunStateMachine>::SharedPtr action_server_;
  std::string default_state_machine_file_;
  bool enable_viewer_pub_{true};

  mutable std::mutex state_mutex_;
  bool busy_{false};
  bool cancel_requested_{false};
  bool shutting_down_{false};
  std::shared_ptr<YasminFactory> active_factory_;
  yasmin::StateMachine::SharedPtr active_state_machine_;
  std::thread execution_thread_;
};

} // namespace yasmin_factory

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  yasmin_ros::set_ros_loggers();

  {
    auto node = yasmin_ros::YasminNode::get_instance();
    yasmin_factory::YasminFactoryActionServer server(node);

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  yasmin_ros::YasminNode::destroy_instance();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
