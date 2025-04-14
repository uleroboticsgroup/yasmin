#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/bar_state.h"
#include "yasmin_demos/foo_state.h"
#include "yasmin_ros/ros_logs.hpp"

using namespace yasmin;

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("multiple_states_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("FOO", std::make_shared<FooState>(),
                {
                    {"outcome1", "BAR"},
                    {"outcome2", "outcome4"},
                });
  sm->add_state("BAR", std::make_shared<BarState>(),
                {
                    {"outcome3", "FOO"},
                });

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
