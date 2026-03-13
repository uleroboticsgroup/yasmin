#include <iostream>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();
  YASMIN_LOG_INFO("yasmin_ros_serialization_demo");

  // Create the factory and state machine in a scope to ensure proper cleanup
  yasmin_factory::YasminFactory factory;

  // Load state machine from XML file
  std::string xml_file =
      ament_index_cpp::get_package_share_directory("yasmin_demos") +
      "/state_machines/demo_3.xml";

  // Create the state machine from the XML file
  auto sm = factory.create_sm_from_file(xml_file);
  sm->set_sigint_handler(true);

  // Publisher for visualizing the state machine
  yasmin_viewer::YasminViewerPub yasmin_pub(sm,
                                            "YASMIN_ROS_SERIALIZATION_DEMO");

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
