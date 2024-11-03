/**
 * @file main.cpp
 * @brief Demonstrates a state machine that adds two integers using ROS 2
 * services.
 *
 * This example shows how to set up a state machine in ROS 2 to call a service
 * that adds two integers, retrieves the result, and prints it.
 *
 * The state machine consists of three states:
 * - SETTING_INTS: Initializes two integers on a blackboard.
 * - ADD_TWO_INTS: Calls a service to add the integers.
 * - PRINTING_SUM: Prints the result of the addition.
 *
 * This program is distributed under the GNU General Public License, version 3.
 */

#include <iostream>
#include <memory>
#include <string>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

/**
 * @brief Sets two integer values in the blackboard.
 *
 * Sets the integers "a" and "b" in the blackboard with values 10 and 5,
 * respectively.
 *
 * @param blackboard Shared pointer to the blackboard for setting values.
 * @return std::string Outcome indicating success or failure.
 */
std::string
set_ints(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("a", 10);
  blackboard->set<int>("b", 5);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @brief Prints the sum stored in the blackboard.
 *
 * Retrieves the integer "sum" from the blackboard and prints it.
 *
 * @param blackboard Shared pointer to the blackboard for getting values.
 * @return std::string Outcome indicating success.
 */
std::string
print_sum(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  fprintf(stderr, "Sum: %d\n", blackboard->get<int>("sum"));
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @class AddTwoIntsState
 * @brief State for calling the AddTwoInts service in ROS 2.
 *
 * This state constructs and sends a service request to add two integers, and
 * processes the response to retrieve and store the result in the blackboard.
 */
class AddTwoIntsState
    : public yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> {
public:
  /**
   * @brief Constructor for AddTwoIntsState.
   *
   * Initializes the service state with the specified service name and handlers
   * for request creation and response processing.
   */
  AddTwoIntsState()
      : yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts>(
            "/add_two_ints",
            std::bind(&AddTwoIntsState::create_request_handler, this, _1),
            {"outcome1"},
            std::bind(&AddTwoIntsState::response_handler, this, _1, _2)){};

  /**
   * @brief Creates a service request using values from the blackboard.
   *
   * Retrieves integers "a" and "b" from the blackboard and sets them in the
   * request.
   *
   * @param blackboard Shared pointer to the blackboard for retrieving values.
   * @return example_interfaces::srv::AddTwoInts::Request::SharedPtr The service
   * request.
   */
  example_interfaces::srv::AddTwoInts::Request::SharedPtr
  create_request_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = blackboard->get<int>("a");
    request->b = blackboard->get<int>("b");
    return request;
  };

  /**
   * @brief Handles the service response and stores the result in the
   * blackboard.
   *
   * Retrieves the sum from the service response and stores it in the
   * blackboard.
   *
   * @param blackboard Shared pointer to the blackboard for storing values.
   * @param response Shared pointer to the service response containing the sum.
   * @return std::string Outcome indicating success.
   */
  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {

    blackboard->set<int>("sum", response->sum);
    return "outcome1";
  };
};

/**
 * @brief Main function to initialize and run the state machine.
 *
 * Sets up logging, initializes ROS 2, and defines a state machine with three
 * states.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code indicating success or failure.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_service_client_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 logging.
  yasmin_ros::set_ros_loggers();

  // Create a state machine with a specified outcome.
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel the state machine on ROS 2 shutdown.
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine.
  sm->add_state("SETTING_INTS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    set_ints),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "ADD_TWO_INTS"},
                });
  sm->add_state("ADD_TWO_INTS", std::make_shared<AddTwoIntsState>(),
                {
                    {"outcome1", "PRINTING_SUM"},
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("PRINTING_SUM",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    print_sum),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                });

  // Publish state machine visualization.
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // Execute the state machine.
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
