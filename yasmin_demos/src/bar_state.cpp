#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_demos/bar_state.h"
#include "yasmin_ros/ros_logs.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
BarState::BarState() : yasmin::State({"outcome3"}){};

/**
 * @brief Executes the Bar state logic.
 *
 * This method logs the execution, waits for 3 seconds,
 * retrieves a string from the blackboard, and logs it.
 *
 * @param blackboard Shared pointer to the blackboard for state communication.
 * @return std::string The outcome of the execution: "outcome3".
 */
std::string
BarState::execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  YASMIN_LOG_INFO("Executing state BAR");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

  return "outcome3";
};

BarState::~BarState(){};