
#ifndef YASMIN_VIEWER_PUB_HPP
#define YASMIN_VIEWER_PUB_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

#include "yasmin_msgs/msg/state.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

namespace yasmin_viewer {

class YasminViewerPub {

public:
  YasminViewerPub(rclcpp::Node *node, std::string fsm_name,
                  std::shared_ptr<yasmin::StateMachine> fsm);

  std::vector<yasmin_msgs::msg::Transition>
  parse_transitions(std::map<std::string, std::string> transitions);

  void parse_state(std::string name, std::shared_ptr<yasmin::State> state,
                   std::map<std::string, std::string> transitions,
                   std::vector<yasmin_msgs::msg::State> &states_list,
                   int parent);

protected:
  void start_publisher(std::string fsm_name,
                       std::shared_ptr<yasmin::StateMachine> fsm);

private:
  rclcpp::Node *node;
  std::unique_ptr<std::thread> thread;
};

} // namespace yasmin_viewer

#endif
