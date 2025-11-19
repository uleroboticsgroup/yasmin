// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Pose = geometry_msgs::msg::Pose;

// Constants for state outcomes
const std::string HAS_NEXT = "has_next"; ///< Indicates there are more waypoints
const std::string END = "end";           ///< Indicates no more waypoints

/**
 * @class Nav2State
 * @brief ActionState for navigating to a specified pose using ROS 2 Navigation.
 */
class Nav2State : public yasmin_ros::ActionState<NavigateToPose> {
public:
  /**
   * @brief Constructs a Nav2State object.
   *
   * Initializes the action state with the NavigateToPose action type,
   * action name, and goal creation callback.
   */
  Nav2State()
      : yasmin_ros::ActionState<NavigateToPose>(
            "/navigate_to_pose",
            std::bind(&Nav2State::create_goal_handler, this, _1)) {}

  /**
   * @brief Creates a goal for navigation based on the current pose in the
   * blackboard.
   *
   * @param blackboard Shared pointer to the blackboard instance holding current
   * state data.
   * @return NavigateToPose::Goal The constructed goal for the navigation
   * action.
   */
  NavigateToPose::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    NavigateToPose::Goal goal;
    goal.pose.pose = blackboard->get<Pose>("pose");
    goal.pose.header.frame_id = "map"; // Set the reference frame to 'map'
    return goal;
  }
};

/**
 * @brief Initializes waypoints in the blackboard for navigation.
 *
 * @param blackboard Shared pointer to the blackboard instance to store
 * waypoints.
 * @return std::string Outcome indicating success (SUCCEED).
 */
std::string
create_waypoints(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  std::map<std::string, std::vector<double>> waypoints = {
      {"entrance", {1.25, 6.30, -0.78, 0.67}},
      {"bathroom", {4.89, 1.64, 0.0, 1.0}},
      {"livingroom", {1.55, 4.03, -0.69, 0.72}},
      {"kitchen", {3.79, 6.77, 0.99, 0.12}},
      {"bedroom", {7.50, 4.89, 0.76, 0.65}}};
  blackboard->set<std::map<std::string, std::vector<double>>>("waypoints",
                                                              waypoints);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @brief Selects a random set of waypoints from the available waypoints.
 *
 * @param blackboard Shared pointer to the blackboard instance to store random
 * waypoints.
 * @return std::string Outcome indicating success (SUCCEED).
 */
std::string take_random_waypoint(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  auto waypoints =
      blackboard->get<std::map<std::string, std::vector<double>>>("waypoints");
  int waypoints_num = blackboard->get<int>("waypoints_num");

  std::vector<std::string> waypoint_names;
  for (const auto &pair : waypoints) {
    waypoint_names.push_back(pair.first);
  }

  // Randomly select waypoints_num waypoints
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(waypoint_names.begin(), waypoint_names.end(), g);
  std::vector<std::string> random_waypoints(
      waypoint_names.begin(), waypoint_names.begin() + waypoints_num);

  blackboard->set<std::vector<std::string>>("random_waypoints",
                                            random_waypoints);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @brief Retrieves the next waypoint from the list of random waypoints.
 *
 * Updates the blackboard with the pose of the next waypoint.
 *
 * @param blackboard Shared pointer to the blackboard instance holding current
 * state data.
 * @return std::string Outcome indicating whether there is a next waypoint
 * (HAS_NEXT) or if navigation is complete (END).
 */
std::string
get_next_waypoint(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  auto random_waypoints =
      blackboard->get<std::vector<std::string>>("random_waypoints");
  auto waypoints =
      blackboard->get<std::map<std::string, std::vector<double>>>("waypoints");

  if (random_waypoints.empty()) {
    return END;
  }

  std::string wp_name = random_waypoints.back();
  random_waypoints.pop_back();
  blackboard->set<std::vector<std::string>>("random_waypoints",
                                            random_waypoints);

  auto wp = waypoints.at(wp_name);

  Pose pose;
  pose.position.x = wp[0];
  pose.position.y = wp[1];
  pose.orientation.z = wp[2];
  pose.orientation.w = wp[3];

  blackboard->set<Pose>("pose", pose);
  blackboard->set<std::string>("text", "I have reached waypoint " + wp_name);

  return HAS_NEXT;
}

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_nav2_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create state machines
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::ABORT,
                                         yasmin_ros::basic_outcomes::CANCEL});
  auto nav_sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::ABORT,
                                         yasmin_ros::basic_outcomes::CANCEL});

  // Cancel state machines on ROS 2 shutdown
  rclcpp::on_shutdown([sm, nav_sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
    if (nav_sm->is_running()) {
      nav_sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state(
      "CREATING_WAYPOINTS",
      std::make_shared<yasmin::CbState>(
          std::initializer_list<std::string>{
              yasmin_ros::basic_outcomes::SUCCEED},
          create_waypoints),
      std::map<std::string, std::string>{
          {yasmin_ros::basic_outcomes::SUCCEED, "TAKING_RANDOM_WAYPOINTS"}});
  sm->add_state("TAKING_RANDOM_WAYPOINTS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    take_random_waypoint),
                std::map<std::string, std::string>{
                    {yasmin_ros::basic_outcomes::SUCCEED, "NAVIGATING"}});

  nav_sm->add_state(
      "GETTING_NEXT_WAYPOINT",
      std::make_shared<yasmin::CbState>(
          std::initializer_list<std::string>{END, HAS_NEXT}, get_next_waypoint),
      std::map<std::string, std::string>{
          {END, yasmin_ros::basic_outcomes::SUCCEED},
          {HAS_NEXT, "NAVIGATING"}});
  nav_sm->add_state(
      "NAVIGATING", std::make_shared<Nav2State>(),
      std::map<std::string, std::string>{
          {yasmin_ros::basic_outcomes::SUCCEED, "GETTING_NEXT_WAYPOINT"},
          {yasmin_ros::basic_outcomes::CANCEL,
           yasmin_ros::basic_outcomes::CANCEL},
          {yasmin_ros::basic_outcomes::ABORT,
           yasmin_ros::basic_outcomes::ABORT}});

  sm->add_state(
      "NAVIGATING", nav_sm,
      std::map<std::string, std::string>{{yasmin_ros::basic_outcomes::SUCCEED,
                                          yasmin_ros::basic_outcomes::SUCCEED},
                                         {yasmin_ros::basic_outcomes::CANCEL,
                                          yasmin_ros::basic_outcomes::CANCEL},
                                         {yasmin_ros::basic_outcomes::ABORT,
                                          yasmin_ros::basic_outcomes::ABORT}});

  // Publish FSM information for visualization
  yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_NAV2_DEMO");

  // Execute the state machine
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("waypoints_num",
                       2); // Set the number of waypoints to navigate

  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}